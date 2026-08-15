/*
 * Raw DMA-buf geometry gate regression test.
 *
 * Mirrors the raw_geom rule of odl_tb5_dmabuf_walk() (driver/odl_tb5_ring_dma.c):
 * the raw zero-copy path is eligible only when every cell except the transfer
 * tail is a full cell.  The two importers (sender / receiver) must chunk the
 * payload identically, and the raw wire carries no header to realign them; a
 * short mid-transfer cell at an SG boundary on one host can be a full cell at
 * the same byte offset on the other, so any such cell forfeits eligibility.
 *
 * Real DMA-heap and GPU exporters hand back page-granular SG tables (every
 * entry a multiple of PAGE_SIZE).  A raw cell that does not divide PAGE_SIZE
 * — e.g. the old ODL_TB5_FRAME_LEN_MAX (4032) — leaves a 64-byte residue in
 * every 4K page, so the gate rejects every buffer that spans more than one
 * page and the transfer silently falls back to framed.  Worse, the two hosts
 * allocate independently, so one side can stay raw while the other falls back,
 * and the raw RX length validation then trips (raw_rx_len_mismatch).
 *
 * The fix pins the raw cell to ODL_TB5_RAW_CELL_MAX (2048), a power of two
 * dividing PAGE_SIZE: every SG boundary then coincides with a cell boundary,
 * all non-tail cells are full by construction, and the chunking is
 * deterministic in `len` alone — both hosts agree without inspecting each
 * other's scatterlist.
 *
 * This test re-derives the walk's cell cutting over synthetic page-granular
 * layouts and asserts the invariant.  It runs on a single box with no
 * kernel module or peer; it is not a substitute for the two-host readiness
 * gate (scripts/odl_tb5_dmabuf_readiness.sh).
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "odl_tb5_uapi.h"

#define TEST_PAGE_SIZE 4096
#define MAX_ENTRIES    8192

/* Mirrors odl_tb5_dmabuf_walk()'s cell cutting for the raw_geom rule only
 * (no ring-occupancy limit, no path selection — those do not affect cell
 * sizes).  Fills *raw_geom like the kernel does: cleared when any cell other
 * than the transfer tail is shorter than chunk_max.  Returns 0 when the
 * layout covers offset+len, -1 otherwise. */
static int walk_raw_geom(const size_t *entries, int n_entries, size_t offset,
			 size_t len, size_t chunk_max, int *raw_geom)
{
	size_t skip = offset;
	size_t total_remaining = len;
	int e;

	*raw_geom = 1;
	for (e = 0; e < n_entries && total_remaining > 0; e++) {
		size_t count_remaining = entries[e];

		if (skip > 0) {
			if ((size_t)skip >= count_remaining) {
				skip -= count_remaining;
				continue;
			}
			count_remaining -= skip;
			skip = 0;
		}

		while (count_remaining > 0 && total_remaining > 0) {
			size_t chunk = chunk_max;

			if (count_remaining < chunk)
				chunk = count_remaining;
			if (total_remaining < chunk)
				chunk = total_remaining;
			if (*raw_geom && chunk < chunk_max &&
			    chunk < total_remaining)
				*raw_geom = 0;
			count_remaining -= chunk;
			total_remaining -= chunk;
		}
	}
	return total_remaining == 0 ? 0 : -1;
}

/* Build the scatterlist a page-granular exporter hands back for a `size`-byte
 * buffer: every entry a full page, like /dev/dma_heap/system. */
static void build_page_layout(size_t size, size_t entries[], int *n)
{
	*n = 0;
	while (size > 0 && *n < MAX_ENTRIES) {
		entries[(*n)++] = TEST_PAGE_SIZE;
		size -= size >= TEST_PAGE_SIZE ? TEST_PAGE_SIZE : size;
	}
	if (size > 0) {
		fprintf(stderr, "layout too large for test table\n");
		exit(2);
	}
}

static int failures;

#define CHECK(cond, ...)					\
	do {							\
		if (!(cond)) {					\
			fprintf(stderr, "FAIL: " __VA_ARGS__);	\
			fprintf(stderr, "\n");			\
			failures++;				\
		}						\
	} while (0)

/* One layout/offset/len/cell-size verdict. */
static void check_geom(const char *what, size_t offset, size_t len,
		       size_t cell, size_t entries[], int n_entries,
		       int expect_raw)
{
	int raw_geom = 0;
	int rc = walk_raw_geom(entries, n_entries, offset, len, cell, &raw_geom);

	CHECK(rc == 0, "%s: walk did not cover offset+len", what);
	CHECK(raw_geom == expect_raw,
	      "%s: offset=%zu len=%zu cell=%zu raw_geom=%d expected %d",
	      what, offset, len, cell, raw_geom, expect_raw);
}

int main(void)
{
	size_t entries[MAX_ENTRIES];
	int n_entries;

	/* ── gate sizes from the readiness gate, page-granular layout ───── */
	{
		static const size_t sizes[] = { 4096, 65536, 1048576, 4194304 };
		size_t i;

		for (i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
			build_page_layout(sizes[i], entries, &n_entries);
			check_geom("readiness-size raw", 0, sizes[i],
				   ODL_TB5_RAW_CELL_MAX, entries, n_entries, 1);
		}
	}

	/* ── the pre-fix bug: 4032 cells fail any multi-page buffer ──────── */
	{
		static const size_t sizes[] = { 4096, 65536, 1048576, 4194304 };
		size_t i;

		for (i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
			build_page_layout(sizes[i], entries, &n_entries);
			/* single page stays eligible (tail is the residue);
			 * anything larger rejects */
			check_geom("legacy-4032 single-page", 0, sizes[i],
				   ODL_TB5_FRAME_LEN_MAX, entries, n_entries,
				   sizes[i] <= TEST_PAGE_SIZE ? 1 : 0);
		}
	}

	/* ── odd lengths / offsets stay raw-eligible at the new cell ─────── */
	{
		static const size_t lens[] = { 100, 4097, 65535, 1048577,
					       4194303 };
		size_t i;

		for (i = 0; i < sizeof(lens) / sizeof(lens[0]); i++) {
			build_page_layout(lens[i] + TEST_PAGE_SIZE, entries,
					  &n_entries);
			check_geom("odd-length raw", 0, lens[i],
				   ODL_TB5_RAW_CELL_MAX, entries, n_entries, 1);
		}
		/* page-aligned interior offsets keep the invariant */
		build_page_layout(2 * TEST_PAGE_SIZE, entries, &n_entries);
		check_geom("offset-page-aligned raw", TEST_PAGE_SIZE,
			   TEST_PAGE_SIZE, ODL_TB5_RAW_CELL_MAX, entries,
			   n_entries, 1);
	}

	/* ── mixed-order heap layout (64K block then 4K pages) ───────────── */
	{
		n_entries = 0;
		entries[n_entries++] = 65536;
		while ((size_t)n_entries * TEST_PAGE_SIZE < 1048576)
			entries[n_entries++] = TEST_PAGE_SIZE;
		check_geom("mixed-order raw", 0, 1048576,
			   ODL_TB5_RAW_CELL_MAX, entries, n_entries, 1);
	}

	/* ── sanity: the invariant must fail for a non-power-of-two cell ─── */
	{
		static const size_t bad_cells[] = { 4032, 3968, 1025, 1000 };
		size_t i;

		build_page_layout(65536, entries, &n_entries);
		for (i = 0; i < sizeof(bad_cells) / sizeof(bad_cells[0]); i++)
			check_geom("non-pow2-cell multi-page", 0, 65536,
				   bad_cells[i], entries, n_entries, 0);
	}

	if (failures == 0)
		printf("raw geometry gate OK: RAW_CELL_MAX=%d divides "
		       "PAGE_SIZE; page-granular layouts stay raw-eligible\n",
		       ODL_TB5_RAW_CELL_MAX);
	return failures ? 1 : 0;
}