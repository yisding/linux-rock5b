Rockchip RGA Rewrite ABI
========================

Scope
-----

The rewrite registers ``/dev/rga`` when ``ROCKCHIP_RGA_REWRITE`` is enabled
and the BSP RGA drivers are disabled.  It implements the supported subset of
the current RK3588 ``librga`` ABI for the RGA3 ``3.0.76831`` and RGA2E
``3.2.63318`` cores.

This document describes the userspace contract.  Internal locking, recovery,
KUnit coverage, and implementation history are intentionally kept out of it.
Ioctl numbers, structure layouts, field widths, and enum values remain the
fixed definitions in the public RGA UAPI headers.

Ioctls
------

The native and compat entry points use the same fixed-width parser.

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Operation
     - Behavior
   * - ``RGA_GET_VERSION``
     - Copies a 16-byte ``major.minor`` string for the first bound core, or
       ``"0.00"`` when no core is bound.
   * - ``RGA2_GET_VERSION``
     - Legacy RGA2 query.  An absent RGA2 core returns ``-EFAULT``; success is
       positive, matching BSP/librga.
   * - ``RGA_IOC_GET_HW_VERSION``
     - Reports the validated RGA2E/RGA3 tuples and returns positive success.
   * - ``RGA_IOC_GET_DRVIER_VERSION``
     - Reports ``1.3.11`` and returns positive success.  The misspelled public
       command name is retained.
   * - ``RGA_IOC_IMPORT_BUFFER``
     - Imports session-owned DMA-BUF or userspace virtual buffers and returns
       handles.
   * - ``RGA_IOC_RELEASE_BUFFER``
     - Releases handles.  Requests and jobs keep their references.
   * - ``RGA_IOC_REQUEST_CREATE``
     - Creates a session request and returns its positive ID.
   * - ``RGA_IOC_REQUEST_CONFIG``
     - Replaces staged tasks and acquire fences without submitting work or
       exporting a release fence.
   * - ``RGA_IOC_REQUEST_SUBMIT``
     - Submits and consumes the request.  Its ID is immediately reusable.
   * - ``RGA_IOC_REQUEST_CANCEL``
     - Cancels and releases a staged request.
   * - ``RGA_BLIT_SYNC``
     - Legacy synchronous submission.
   * - ``RGA_BLIT_ASYNC``
     - Legacy asynchronous submission; publishes ``out_fence_fd``.
   * - ``RGA_CACHE_FLUSH``, ``RGA_FLUSH``, ``RGA2_FLUSH``,
       ``RGA_GET_RESULT``, ``RGA2_GET_RESULT``
     - BSP-compatible no-ops returning success.

``RGA_IMPORT_DMA`` and ``RGA_RELEASE_DMA`` are recognized but return
``-EINVAL``.  Use the modern import/release ioctls.  Unknown commands also
return ``-EINVAL``.

Buffers and addresses
---------------------

Imports are owned by the open file session.

DMA-BUF
  The ABI value must fit exactly in a nonnegative ``int`` fd.  The mapped
  attachment must cover the complete imported DMA-BUF as one byte-contiguous
  span in the 32-bit RGA command-address aperture.  Multiple mapped entries
  are accepted only when their DMA addresses are byte-adjacent.  A genuinely
  gapped mapping is representable through an eligible RGA2 page table; it must
  force that RGA2 core mask because a later RGA3 mapping rejection is not
  retried on RGA2.

Userspace virtual address
  Pages are long-term pinned and synchronized around hardware access.  When
  the ordinary DMA mapping is gapped, an RGA3 core with an IOMMU may receive a
  driver-owned contiguous IOVA.  RGA2 uses its page-table path when the mapped
  pages are representable.  CPU writes made between jobs remain visible;
  final release does not overwrite CPU-owned data; and cache maintenance or
  shadow copyback does not alter bytes outside an unaligned imported range.

Legacy direct buffers
  MMU-backed DMA-BUF fd and userspace-virtual channels are supported through
  temporary job-owned imports.  The current wire format has no explicit
  discriminator: a nonzero primary address no larger than ``INT_MAX`` is
  interpreted as a DMA-BUF fd, so fd-shaped low virtual addresses are not
  supported.  Direct physical-address channels are rejected.

A request may mix DMA-BUF and USERPTR channels only when every page pinned
for each USERPTR channel is anonymous.  Anonymous pages come from no DMA-BUF
exporter reachable on this platform, so overlap is excluded without having to
prove it across the exporter boundary.  A USERPTR channel backed by anything
else -- a file mapping, shmem, or a mapped DMA-BUF -- keeps the whole request
rejected, because overlap then cannot be proved either way.

Import and release pools contain at most 40 entries; larger pools return
``-EFBIG``.  Import is transactional, including copyout rollback.  Release is
sequential: if a later handle is invalid, earlier handles in that call remain
released.  A released handle may stay mapped while a request or job retains
it.  Modern physical-address import types return ``-EOPNOTSUPP``.  A DMA-BUF
fd value that cannot be represented losslessly returns ``-EINVAL``.

The driver does not wait on or publish DMA-BUF reservation fences.  Callers
must serialize buffer reuse themselves, using explicit RGA acquire/release
sync files where applicable.

Handle-mode planes
------------------

``uv_addr`` is the explicit-plane discriminator.

- ``uv_addr == 0`` means one imported handle.  UV/V addresses are derived from
  the validated image layout.  A lone nonzero ``v_addr`` is treated as a
  librga placeholder and cleared.
- ``uv_addr != 0`` supplies an explicit UV handle.
- A nonzero ``v_addr`` is an explicit V handle only when ``uv_addr != 0``.

Every required plane and active rectangle must fit its retained import.
Compressed and tiled images are sized from their render-mode layout, including
headers and payloads.

Requests, jobs, and fences
--------------------------

``REQUEST_CONFIG`` copies tasks, Gaussian coefficients, handle references, and
acquire fences into kernel-owned storage.  Reconfiguring replaces the previous
staged resources.  It may stage a profile that no backend supports; profile
validation occurs at submission.  The ID, task pointer, and task count must be
nonzero or ``-EINVAL`` is returned.  At most 256 tasks are accepted; a larger
request returns ``-EFBIG``.

``REQUEST_SUBMIT`` atomically clones and removes the exact request while the
session request lock is held.  Its ID may then be reused without the old submit
removing the new request.  ``REQUEST_CONFIG`` leaves the ID live.  CREATE
copyout failure leaves no ID, and CANCEL of an unknown ID returns ``-EINVAL``.

Synchronous jobs return the final completion status.  Asynchronous jobs return
a sync-file release fence and signal it with the final status.  Fence fd
publication is atomic with respect to userspace copyout: a copy fault cannot
close or expose an unrelated fd.  Modern requests receive it in
``release_fence_fd``; legacy async blits use ``out_fence_fd`` and otherwise
preserve the caller's request fields.  Each job has an independent fence
timeline, so merging out-of-order jobs cannot collapse their completions.
For a ready async job, dispatch may begin before the fence number is copied
out.  Therefore ``-EFAULT`` during that copyout means no fd was published, but
does not guarantee that the blit did not execute.

Acquire fences are retained by the job.  When
``feature.user_close_fence == 0``, the driver closes the submitted acquire-fd
after taking its own fence reference; when it is set, userspace retains close
ownership.  Synchronous submission waits for an unsignaled acquire fence.
Asynchronous submission publishes its release fence while the acquire fence is
pending and dispatches later.  An already-signaled successful acquire fence
takes the ready path.  A negative acquire-fence result becomes the job result.
Kernel-owned acquire fds are closed on both success and preparation failure.
Legacy ``RGA_BLIT_SYNC``/``RGA_BLIT_ASYNC`` requests follow the BSP sentinel
rule: ``in_fence_fd == 0`` means that no acquire fence was supplied.  Modern
``REQUEST_*`` submissions apply the same rule to the request-level
``acquire_fence_fd`` because zero is the public ``imendJob()`` default and the
BSP imports only positive request-fence descriptors.  The normalization
changes only the kernel-owned fence lookup; asynchronous replies preserve the
caller's zero field.  Task-local ``in_fence_fd`` fields in a modern request
retain normal fd-zero semantics.

Closing ``/dev/rga`` prevents new work, cancels acquire-blocked work, removes
queued work, recovers active work, signals outstanding release fences, and
then releases requests and imports.  Close completes unfinished work with
``-EFAULT``; hardware removal uses ``-ENODEV``.  If reset cannot prove that an
active DMA master stopped, close/unbind retries while retaining the active job,
mappings, command storage, power, and unsignaled release fence.

Scheduling
----------

Tasks are validated against the selected backend before hardware starts.
Without a forced mask, the exact compatible core choice is implementation
defined; the current scheduler balances load and rotates equal-load ties.  BSP
core-mask bits are honored:

=========  ================
Bits       Class
=========  ================
``0x1``    RGA3 core 0
``0x2``    RGA3 core 1
``0x4``    RGA2 core 0
``0x8``    RGA2 core 1
=========  ================

Work is never rerouted outside a forced mask; a mask with no present compatible
core fails.  Priorities are clamped to 0..6.  Higher nonzero priority precedes
lower queued priority, and displaced work ages toward 6.  One job runs per
core.  Multi-task requests are validated as a whole and run serially under one
completion/fence; individual tasks may use different compatible core classes.

A reset failure quarantines the core.  Its queued jobs and acquire-blocked work
that no longer has a compatible core complete with ``-EIO``.  Removal prevents
new admission, aborts queued/active work after proving DMA quiescence, and
allows reprobe to reuse the vacant public core bit.

Supported operation profiles
----------------------------

These tables summarize the accepted RK3588 profiles.  The format lists and the
constraints below are part of the boundary; fields not used by a listed
profile must be zero/default.

====================  ========================================================
RGA3                  Supported subset
====================  ========================================================
Raster formats        RGBA/RGBX/BGRA/BGRX/ARGB/XRGB/ABGR/XBGR 8888;
                      RGB/BGR 888 and 565; YVYU/VYUY/YUYV/UYVY 422; YCbCr and
                      YCrCb semiplanar 420/422 8-bit and 10-bit.  ARGB/XRGB/
                      ABGR/XBGR are input-only.
Raster operations     Crop, destination offset, scale, format conversion,
                      interpolation selectors, rotate, and mirror.  Source
                      YUV422 cannot rotate 90/270 degrees.
AFBC16x16              The RGB and semiplanar formats above, except packed
                      YUV422.  ARGB/XRGB/ABGR/XBGR remain decode-only.
Tile8x8               Simple raster/tile/tile copies of semiplanar YCbCr/YCrCb
                      420/422 8-bit or 10-bit; no alpha, pattern, or color key.
CSC                   Native limited/full-range selector modes.  Full-CSC
                      compatibility is only ``full_csc.flag`` enabled,
                      ``yuv2rgb_mode == (3 << 2)``, RGB source, and YUV
                      destination; the coefficient block is ignored and
                      native BT.709 limited conversion is used.
Alpha                 A+B composition for the listed RGB and semiplanar paths.
                      Pattern blend requires alpha, equal pattern/destination
                      size, and no pattern-image or main-image transform.
                      Porter-Duff:
                      SRC, DST, SRC_OVER, DST_OVER, SRC_IN, DST_IN, SRC_OUT,
                      DST_OUT, SRC_ATOP, DST_ATOP, XOR, and CLEAR.
Color key             Two-image raster RGB only.  Normal and inverted
                      selectors intentionally use the same BSP top-key rule.
Border/copy splice    Nonoverlapping same-buffer mirror copies and serial
                      destination-rectangle splice tasks.
====================  ========================================================

====================  ========================================================
RGA2E                 Supported subset
====================  ========================================================
Raster formats        8888/888/565/5551/4444 RGB families; packed YUV422;
                      planar and semiplanar YCbCr/YCrCb 420/422; semiplanar
                      444; Y400; compact 10-bit semiplanar input.  Packed
                      YUV420, Y4, and Y8 are output-only.
Raster operations     Crop, offset, scale, conversion, interpolation, rotate,
                      mirror, supported full-CSC RGB-to-YUV/gray, and Y4/Y8
                      dither.  BPP formats are not raster bitblit inputs.
Fill                  RGB and supported 8-bit planar, semiplanar, and packed
                      YUV.  YUV fill requires a valid RGB-to-YUV mode in
                      ``yuv2rgb_mode`` bits 2..4.
Palette               Two-command LUT update then conversion.  Modes 0..3 map
                      BPP1/2/4/8 (or Y400 for mode 3) through an optional
                      16x16 RGBA8888 LUT into a supported RGB destination.
Special operations    RGB mosaic; ROP2 AND, OR, NOT_DST, NOT_SRC, XOR, and
                      NOT_XOR; 3x3 RGB Gaussian blur; RGB NN quantize;
                      RGBA5551 alpha bitmap; fixed-width RGB OSD; RGBA color
                      key; and pre-interrupt on an otherwise supported job.
====================  ========================================================

Gaussian, quantize, ROP, color-key, alpha-bitmap, OSD, and palette conversion
use raster images with the profile-specific same-size/no-transform limits.
Line-only RGA2 pre-interrupts do not complete a job.

Format rules
------------

- Raster and tile semiplanar 10-bit ``vir_w`` is a byte stride.  FBC ``vir_w``
  remains a pixel count.  ``act_w`` and offsets are pixels.
- YUV rectangles and plane offsets must obey the format's chroma alignment.
- RGA3 active windows must meet the RK3588 68x2 floor and per-axis 1/8x..8x
  scale range.  Eligible jobs may fall back to RGA2E, whose floor is 2x2 and
  range is 1/16x..16x.
- In-place work is limited to explicitly supported nonoverlapping mirror/border
  copies, RGA2 mosaic, and destination-update blend/color-key profiles.

Not supported or guaranteed
---------------------------

- Direct physical-address execution.
- Secure/protected-buffer semantics are not guaranteed.
- Mixed DMA-BUF/USERPTR requests whose USERPTR side is not wholly anonymous.
- Genuinely gapped RGA3 DMA-BUF attachments.
- RFBC64x4 and AFBC32x8 execution on RK3588; AFBC32x8/RFBC destination modes;
  packed-YUV FBC; compressed in-place writeback.
- General pattern, mask, ROP4, per-channel rotation, arbitrary full-CSC,
  arbitrary OSD/Gaussian/quantize combinations, or operation modes not listed
  in the support tables.
- In-place scale, conversion, or rotation.

Error behavior
--------------

Legacy blit ioctls preserve the internal validation errno, commonly
``-EOPNOTSUPP`` for an unsupported profile.

After structural request validation succeeds, CONFIG preparation and SUBMIT
preparation/profile failures are normalized to the BSP wrapper's ``-EFAULT``.
A rejected mixed-provenance modern preparation is therefore observed as
``-EFAULT``.  The underlying cause remains in rewrite debug events.

Hardware/lifecycle results include ``-ENODEV`` for lost hardware, ``-EIO`` for
IOMMU fault or work rejected by a quarantined core, ``-EBUSY`` for a watchdog
timeout that did not latch completion, and ``-EACCES`` for decoded RGA2 command
errors.  A watchdog that finds a latched completion returns that IRQ result.
Session teardown signals unfinished async work with ``-EFAULT``.

Failure paths release resources only after the job is known not to be active.
If reset does not prove DMA quiescence, DMA-visible resources and the active
job are retained while stop is retried.  An async fence-copyout fault is the
documented exception to submission rollback: no fd is installed, but already
admitted work may execute.

There is no per-session byte/count quota beyond the per-ioctl limits above.
Imports, USERPTR pins, requests, acquire-blocked work, and unconsumed jobs can
therefore grow until explicit release, cancel, completion, or close.

Validation status
-----------------

The listed profiles and multi-SG paths are implemented, source-audited, and
have compiled KUnit coverage.  They become hardware-qualified only after the
corresponding booted KUnit and official librga runs complete on the target
kernel and board.

Debugging
---------

``/sys/kernel/debug/rk_rga_rewrite`` exposes counters, recent ``events``, and
``trace_mask``.  Important live gauges include ``import_count`` for explicit
session handle imports, command/fence/job counters, per-core scheduling and
timing, IOMMU faults, timeouts, recovery failures, USERPTR boundary shadows,
and ``route_b`` fallback activity.  Job-owned legacy/direct imports are not
included in ``import_count``.

Debugfs is diagnostic and is not a stable userspace ABI.
