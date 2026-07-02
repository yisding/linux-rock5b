Rockchip RGA Rewrite ABI Status
===============================

This rewrite registers ``/dev/rga`` when ``ROCKCHIP_MULTI_RGA`` is disabled and
``ROCKCHIP_RGA_REWRITE`` is enabled.

Implemented
-----------

* Legacy ``RGA_GET_VERSION`` and ``RGA2_GET_VERSION`` query paths.
* Modern ``RGA_IOC_GET_HW_VERSION`` and ``RGA_IOC_GET_DRVIER_VERSION``.
  Hardware version queries report the RK3588-compatible RGA2E
  ``3.2.63318`` and RGA3 ``3.0.76831`` tuples used by current ``librga``
  capability probing.
* RGA2/RGA3 platform-driver binding for RK3588 BSP and mainline compatibles,
  with devm-managed MMIO, IRQ, clock, and reset discovery.
* ``RGA_IOC_IMPORT_BUFFER`` and ``RGA_IOC_RELEASE_BUFFER`` for dma-buf fd and
  userspace virtual-address imports owned by the open file session.  Imported
  dma-bufs are attached and mapped against a bound RGA hardware device through
  public dma-buf APIs.  Virtual-address imports pin user pages, build
  sg_tables through ``sg_alloc_table_from_pages()``, map them with the public
  DMA API, and synchronize them around hardware execution for the common
  CPU-buffer ``librga`` sample paths.  The rewrite prefers an RGA3 node for
  imports because that is the first executable hardware backend; RGA2 fallback
  jobs create job-owned remaps against the selected RGA2 device.
* ``RGA_IOC_REQUEST_CREATE``, ``RGA_IOC_REQUEST_CONFIG``, and
  ``RGA_IOC_REQUEST_CANCEL`` request lifetime management.
* Modern request task-array copy into session-owned request objects.  Handle
  backed image addresses are resolved to mapped IOVAs in the kernel-owned task
  copy, and configured requests hold references to their imported dma-bufs.
  After ``rga_request_check()`` accepts a modern request, config/submit
  preparation failures are normalized to the BSP ioctl wrapper's ``-EFAULT``.
* Legacy blit task copy and acquire-fence fd validation.
* Legacy no-handle blit/fill submissions from ``wrapbuffer_fd()`` and
  ``wrapbuffer_virtualaddr()``.  The rewrite accepts MMU-backed direct fd and
  userspace-virtual channels, converts them into job-owned temporary imports,
  and feeds the existing IOVA-based scheduler/backend path.  Direct physical
  address channels remain unsupported.
* Acquire-fence sync-file validation and ownership for modern request and
  legacy blit paths.  Configured requests hold references to the imported
  ``dma_fence`` objects for submit and close kernel-owned acquire-fence fds
  once those references have been taken.
* ``librga`` acquire-fence ownership semantics.  When a submitted task clears
  ``feature.user_close_fence``, the rewrite closes the imported acquire-fence fd
  after taking its own ``dma_fence`` reference, matching the forward-port
  compatibility path for older userspace.  When the flag is set, userspace keeps
  fd-close ownership.
* Imported-buffer handle validation and IOVA resolution for handle-based task
  channels.
* Render-mode-aware imported-buffer resolution for prepared jobs.  Required
  source/destination/pattern channels are checked against their imported
  dma-buf size, and single-handle multi-plane images get kernel-derived UV/V
  IOVA addresses in the task copy.
* Prepared-job resource ownership for modern submit and legacy blit paths.  A
  submit clones the configured task payload, takes independent references to
  resolved imports and acquire fences, and carries those resources to the
  scheduler/backend boundary.
* Rewrite-local ``dma_fence`` context and prepared-job completion path.  Async
  jobs own a release fence internally and signal it with the same completion
  status that the submit path returns.  If userspace drops a pending async job
  before completion, cleanup signals the release fence with ``-EFAULT`` like
  the BSP request teardown path.
* Async pending-acquire handling for modern submit and legacy blit paths.  If
  an async job is blocked by an unsignaled acquire fence, the ioctl exports the
  release-fence fd, arms ``dma_fence`` callbacks, queues dispatch work when the
  acquire fences have signaled on the high-priority system workqueue like the
  forward port, and signals the release fence with the eventual backend result.
* Ready async jobs for supported hardware profiles now export the release-fence
  fd, queue the prepared job, and return without waiting for IRQ/timeout
  completion.  The queued job owns its own lifetime reference until completion
  signals the release fence.
* Minimal per-core scheduler boundary: submit paths queue prepared jobs on the
  least-loaded eligible bound RGA core, dispatch one active job per core, wake
  submit waiters on completion, and keep hardware nodes alive until in-flight
  scheduler users drain during remove.
  Hardware removal stops new dispatch, completes queued and active jobs with
  ``-ENODEV``, and signals any exported async release fence with that result.
* Backend-aware core selection for prepared jobs.  The scheduler checks the
  same supported-operation profile used by command generation, selects an RGA3
  core for the current hardware-backed profile, and requires that core to match
  the device that owns the job's dma-buf mappings.
  BSP-compatible ``rga_req.core`` scheduler masks are honored for supported
  profiles: RGA3 core bits ``0x1``/``0x2`` select RGA3 cores and RGA2 bits
  ``0x4``/``0x8`` select RGA2 cores.  Unsupported profiles requested on a
  forced core still fail with the normal validation error.  Imported images are
  rebound to the selected core's DMA device at dispatch time, so direct
  ``wrapbuffer_fd()`` submissions can target a non-default RGA core.
* Runtime PM and clock-bulk sequencing around the backend dispatch boundary.
  Each dispatched job resumes the selected RGA core, enables discovered clocks,
  enters the backend, then disables clocks and drops runtime PM.
* Threaded IRQ registration and active-job completion scaffolding.  A backend
  can now accept a job, leave it active, and have the IRQ thread complete it,
  signal fences, release runtime PM/clocks, and dispatch the next queued job.
  The top half decodes RGA2/RGA3 done and error interrupt status, clears handled
  bits, and propagates hardware error status to the job completion result.
* Master-mode RGA2/RGA3 hardware start helpers for generated command buffers.
  Accepted jobs enable done/error interrupts, program the selected core's command
  DMA address, start command execution, and arm a per-core timeout worker.
* Per-core timeout completion for accepted hardware jobs.  Timed-out jobs read
  RGA2/RGA3 status registers, reset the selected core with the BSP-style
  soft-reset helper plus reset-controller fallback when present, complete with
  ``-EBUSY``, release runtime PM/clocks, and dispatch the next queued job.
* Public IOMMU fault callback registration for bound RGA cores.  A fault
  records ``iommu_fault_count`` in debugfs, logs the IOVA/status, marks the
  active job for immediate recovery through the same serialized reset path,
  completes the job with ``-EIO``, and signals any exported async release
  fence with that result.
* Per-job DMA-coherent command-buffer allocation and lifetime, sized for the
  selected RGA2/RGA3 core and released through normal job teardown.
* RGA3 command-buffer generation path for validated raster and AFBC16x16
  bitblits.  The no-blend profile maps source to WIN0 and destination to WR,
  requires imported dma-buf backed source/destination images, rejects color-key
  and unsupported pattern operations, and supports the 8-bit RGB/YUV plus
  semiplanar 10-bit YUV formats exposed by common ``librga`` and
  ``ffmpeg-rockchip`` blit/scale/convert users.  AFBC is accepted through
  ``RGA_FBC_MODE`` for the RGA3 FBCD/FBCE format subset; RFBC, AFBC32x8, tile
  modes, packed-YUV FBC, destination offsets, and compressed in-place alpha
  write-back remain unsupported.  Main request
  rotation/mirror flags are translated to RGA3 WIN0 rotate/mirror controls;
  per-channel rotate flags remain unsupported.  Destination rectangle offsets
  are supported by biasing WR plane base addresses for 8-bit formats;
  semiplanar YUV offsets must be chroma-aligned.
* RGA3 Porter-Duff alpha blend for the common A+B composition paths emitted by
  ``librga`` and ``ffmpeg-rockchip``.  Source/foreground is programmed through
  WIN1.  No-pattern A+B->B uses destination/background through WIN0 and writes
  back to the destination.  Pattern-backed A+B->C accepts ``bsfilter_flag`` with
  an imported pattern/source1 image, programs that image through WIN0, and writes
  WR to the destination.  Alpha flags ``0x19`` plus the non-premultiplied bit 9
  are accepted with per-pixel/global alpha selection.  This subset supports
  RGB-destination composition, including ffmpeg
  ``overlay_rkrga=format=<rgb>`` pipelines, and the default ffmpeg/RKMPP
  overlay path where an 8-bit YUV main/foreground image and RGB/RGBA pattern
  write an 8-bit YUV destination through the per-window CSC policy generated by
  ``librga``.  Main-request rotation/mirror is applied to the foreground WIN1
  path, including BSP-matched 90-degree sizing for pattern and no-pattern A+B
  jobs.  10-bit YUV-destination alpha, no-pattern YUV A+B->B, and compressed
  in-place write-back remain unsupported.
* RGA2 solid color fill for the common ``librga`` ``imfill`` destination-only
  path.  The rewrite selects an RGA2-class RK3588 core, creates job-owned
  dma-buf mappings for that core when the imported handle was originally mapped
  elsewhere, and emits a minimal raster RGB-family fill command.  Destination
  rectangle offsets are applied by biasing the fill destination base address,
  covering the ``imfill`` and decomposed ``imrectangle``/``imrectangleTask``
  paths used by current ``librga`` samples.  This is deliberately an RGA2
  profile: the RK3588 forward-port RGA3 capability table does not advertise
  ``RGA_COLOR_FILL``, so requests forced to RGA3 core bits fail with
  ``-EOPNOTSUPP``.  Pattern fill, alpha/ROP/color-key, rotation, tile/FBC, and
  YUV fill variants remain unsupported.
* RGA2 raster bitblit for the common upstream-consumer fallback formats that
  RGA3 does not cover: planar YUV420/YUV422, YCbCr400/gray, NV24/NV42-style
  YUV444 semiplanar, compact 10-bit semiplanar source, RGB555-family, and
  ARGB/ABGR output.  This path supports imported dma-buf backed
  source/destination images, single-handle and per-plane-handle images, linear
  raster scaling/conversion, BSP-compatible ``rga_req.rotate_mode`` plus
  ``sina``/``cosa`` rotation and mirror encoding, including the active-rectangle
  swap used by ``librga`` for 90/270-degree rotation, the BSP force-tile scale
  mode used by no-scale compact 10-bit and YUV444 semiplanar blits, and
  job-owned dma-buf mappings for the selected RGA2 core.  RGA2 destination
  full-CSC coefficients generated by ``librga`` for RGB-to-YUV conversion are
  accepted for this raster bitblit path and programmed through the
  BSP-compatible CSC register block.  Pattern, color-key, alpha/ROP, tile/FBC,
  incompact 10-bit input, 10-bit output, OSD, mosaic, pre-intr, and gauss
  variants remain unsupported.
* Multi-task requests are accepted when every task matches the same supported
  backend profile; tasks run serially under the request's single
  completion/fence.  The RGA3 no-blend emitted command includes overlap field
  and alpha/default-global-alpha controls.
* Legacy ``RGA_CACHE_FLUSH``, ``RGA_FLUSH``, ``RGA_GET_RESULT``, and
  ``RGA2_GET_RESULT`` as BSP-compatible no-ops.
* Optional ``ROCKCHIP_RGA_REWRITE_KUNIT_TEST`` coverage for rewrite-local ABI
  normalization helpers, including the RGA2 ``rotate_mode``/``sina``/``cosa``
  decoder, transformed destination-corner selection, color-fill core-mask
  dispatch, BSP request task-count limits and return codes, mixed RGA2/RGA3
  multi-task rejection, RGA2 fill destination-offset emission and multi-fill
  task acceptance, compact 10-bit RGA2 source dispatch/emission including the
  no-scale force-tile mode, IOMMU fault target matching, and ffmpeg-facing RGA3
  raster/FBC/alpha-overlay profile selection plus destination-offset command
  emission, source-crop command emission for RGA3 and RGA2, and semiplanar
  chroma-alignment rejection.

Recognized But Unsupported
--------------------------

* RGA2 hardware command generation outside the solid color fill and raster
  bitblit profiles above, including full-CSC outside raster bitblit.
* RGA3 pattern outside the supported alpha-overlay profile, color-key,
  no-pattern or 10-bit YUV-destination alpha, per-channel rotation,
  RFBC/tile/AFBC32x8, 10-bit or AFBC destination offsets,
  virtual/physical-address, and non-bitblit operation modes.
* Mixed RGA2/RGA3 multi-task requests.

Unsupported submit profiles return ``-EOPNOTSUPP`` after copying, validating,
preparing, queuing, dispatching, resolving imported buffers, allocating an owned
command buffer, and power-sequencing an owned job to the backend boundary.

Outside This Slice
------------------

* Physical address imports.
* Full RGA2/RGA3 command-register generation and policy selection.
* Full BSP timeout diagnostics and private-IOMMU recovery after faults beyond
  immediate reset/abort.
