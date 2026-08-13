# Troubleshooting

## Kernel Module

| Problem | Solution |
|---------|----------|
| Build fails with unknown GCC flags | GCC is too old. Install version matching kernel (`cat /proc/version`) |
| Module won't load | Check `dmesg | grep odl_tb5`. Ensure TB5 hardware is present (`lspci | grep Thunderbolt`) |
| No `/dev/odl_tb5_*` devices | Device appears only when a TB5 peer connects. Check `dmesg` for XDomain events. Use `loopback=1` for no-cable testing |
| Permission denied | Install udev rule or `sudo chmod 660 /dev/odl_tb5_*` |
| Probe fails with `DMA buf alloc failed` / `-12` | Host is likely booted with `iommu=pt` — the NHI then sits in an identity IOMMU domain and `dma_alloc_coherent` must return physically contiguous memory. The default `ring_size=4096` needs 16 MB order-12 blocks, which usually fail. The driver now retries with smaller sizes automatically; you can also `modprobe odl_tb5 ring_size=1024` (4 MB, order-10) |

## Daemon & Tray

| Problem | Solution |
|---------|----------|
| Daemon won't start | Check `journalctl --user -u odl-tb5-daemon` for D-Bus errors |
| Tray icon not visible | Install `gnome-shell-extension-appindicator` on GNOME/Wayland |

## Verbs Provider

| Problem | Solution |
|---------|----------|
| `ibv_devinfo` doesn't show ODL device | Provider plugin not installed. Check `/usr/lib/*/libibverbs/` for `libodl_tb5-rdmav34.so` |
| `ibv_open_device` fails | Check `ODL_VERBS_DEBUG=5` for details. Ensure module loaded |
| `ibv_post_send` returns `-EAGAIN` | Normal for non-blocking mode. Worker thread retries after poll |

## GRUB

If TB5 ports aren't working reliably, add to kernel command line:
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash pcie_port_pm=off"
```

Note: some frameworks (e.g. RCCL) print `Missing "iommu=pt"` warnings and
recommend booting with it. On virtualisation hosts that is useful — but
`iommu=pt` puts Thunderbolt's NHI in an identity IOMMU domain, which makes
large `dma_alloc_coherent` buffers (ODL default `ring_size=4096`, 16 MB)
unallocatable. The driver now downgrades automatically; if you hit the
old `DMA buf alloc failed ... -12` symptom, load with `odl_ring_size=1024`.

## Debug

```bash
# Kernel driver debug
echo 'module odl_tb5 +p' | sudo tee /sys/kernel/debug/dynamic_debug/control
dmesg -w | grep odl_tb5

# Verbs provider trace
export ODL_VERBS_DEBUG=5

# Daemon foreground with verbose output
./build/daemon/odl_tb5_daemon -f

# RCCL debug
export RCCL_DEBUG=INFO

# NCCL debug
export NCCL_DEBUG=INFO
```
