# .ci — 设备侧内核 ABI 快照

CI 要产出**能在这台盒子上 `insmod` 成功**的 `.ko`，光有源码不够，还差两样东西：

| 文件 | 作用 |
|---|---|
| `config-6.12.115-dvbip-1002` | 设备上运行中内核的**实际** `.config`（从 `/usr/src/linux-headers-$(uname -r)/.config` 取）。用 defconfig 重新生成有偏差风险，`CONFIG_MODVERSIONS` + 结构体布局都依赖它。 |
| `Module.symvers` | 同一份头文件包里带的内核符号 CRC 表（11455 条）。`modpost` 用它给模块的 `__kcrctab` 打 CRC；不提供的话编出来的模块会因为 `disagrees about version of symbol module_layout` 被拒载。 |

因为 CRC 来自这张表，**CI runner 用什么编译器都不影响能否加载**（设备内核是 clang 19 编的，runner 上是 clang 18）。

## 重新采集（内核升级或改了 config 之后）

```sh
H=/usr/src/linux-headers-6.12.115-dvbip-1002
scp root@192.168.18.179:$H/.config        .ci/config-6.12.115-dvbip-1002
scp root@192.168.18.179:$H/Module.symvers .ci/Module.symvers
```

然后同步改掉 `.github/workflows/build-histb-ko.yml` 里的 `KERNEL_RELEASE`。

## 版本字符串的坑

设备的 `uname -r` 是 `6.12.115-dvbip-1002`，但 `.config` 里 `CONFIG_LOCALVERSION=""` —— 后缀其实来自构建树里的 `localversion-dvbip` 文件。CI 里改成写进 `CONFIG_LOCALVERSION`，并额外 `export LOCALVERSION=`（设成空但存在），否则 `scripts/setlocalversion` 会给 dirty tree 追加 `+`，vermagic 就对不上了。

## 编译器必须用 clang 19

`/proc/version` 显示设备内核是 `Debian clang version 19.1.7 / Debian LLD 19.1.7` 编的。CI 一开始用 Ubuntu 自带的 clang 18，8 个模块里只有一个符号 CRC 对不上：

```
histb-vdec.ko: CRC mismatch for 'histb_vpss_get' (built 0xa05c9d21, device 0x0f7b2f67)
```

查过了，`include/linux/device.h`、`include/linux/pm.h` 与设备 headers **逐字节一致**（md5 相同），所以不是源码漂移，而是编译器预处理差异——`histb_vpss_get(struct device *consumer)` 的签名会把整个 `struct device` 卷进 genksyms 校验和，clang 18/19 对编译器条件属性（`__counted_by` 之类）的展开不同，校验和就变了。换成 apt.llvm.org 的 clang-19 后 CRC 全部吻合，`Collect and verify` 步骤输出 `OK: modules are compatible with the running kernel`。

顺带一个观察：`M=` 方式构建的模块没有 `MODULE_INFO(intree, "Y")`，加载时会打印 `loading out-of-tree module taints kernel` 并打上 taint 标记。功能无影响，介意的话用 `scope=all`（整树 `make modules`）或改成 in-tree 单目标 `make <path>/histb-vdec.ko`。

## patches/

`.ci/patches/*.patch` 会在编译前用 `git apply` 打上（已打过则跳过）。目前只有一支：

- `0001-media-histb-vdec-export-dma-buf-vmap.patch` — 给 `histb-vdec` 导出的 dma-buf 补 `.vmap`/`.vunmap`。缺了它 `dma_buf_vmap()` 直接返回 `-EINVAL`，`drm_gem_dma_prime_import_sg_table_vmap()` 报 `Failed to vmap PRIME buffer`，DRM PRIME 零拷贝整条断掉，E2 只能退回 dumb 拷贝路径。

## 产物

Artifact `histb-ko-<release>-<run>` 里是 `.ko` + `install.sh` + `SHA256SUMS.txt`。盒子上解包后 `./install.sh` 会把每个 ko 放回 `/lib/modules/$(uname -r)/kernel/...` 对应位置并 `depmod -a`。
