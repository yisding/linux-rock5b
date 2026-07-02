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
  ``RGA_IOC_REQUEST_SUBMIT`` consumes the session request after cloning the
  prepared job resources, matching current ``librga`` task-job lifetime.
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
  once those references have been taken.  Preparation failures and legacy
  blit job-allocation failures also release kernel-owned acquire-fence fds
  after dropping the imported fence references.
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
  scheduler users drain during remove.  The per-core queue honors current
  ``librga``/BSP ``rga_req.priority`` values by inserting nonzero-priority
  jobs ahead of lower-priority queued work and aging displaced queued jobs,
  with priorities clamped to the BSP 0..6 range.
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
  soft-reset helper plus reset-controller fallback when present, ask the public
  IOMMU layer to flush the core's attached domain, complete with ``-EBUSY``,
  release runtime PM/clocks, and dispatch the next queued job.
* Public IOMMU fault callback registration for bound RGA cores.  A fault
  records ``iommu_fault_count`` in debugfs, logs the IOVA/status, marks the
  active job for immediate recovery through the same serialized reset path,
  refreshes the attached IOMMU domain after reset, completes the job with
  ``-EIO``, and signals any exported async release fence with that result.
* Per-job DMA-coherent command-buffer allocation and lifetime, sized for the
  selected RGA2/RGA3 core and released through normal job teardown.
* RGA3 command-buffer generation path for validated raster and AFBC16x16
  bitblits.  The no-blend profile normally maps source to WIN0 and destination
  to WR; non-overlapping same-buffer, same-format, no-scale raster mirror copies
  use the BSP-compatible overlap route with source on WIN1, destination/background
  on WIN0, WR on the destination, and the destination rectangle carried in
  ``OVLP_OFF``.  This covers the current ``librga`` ``immakeBorder()`` reflect
  and wrap side-edge tasks.  The path requires imported dma-buf backed
  source/destination images, rejects color-key and unsupported pattern
  operations, and supports the 8-bit RGB/YUV plus semiplanar 10-bit YUV formats
  exposed by common ``librga`` and ``ffmpeg-rockchip`` blit/scale/convert users.
  Explicit interpolation selectors from current ``librga`` resize calls are
  accepted on native RGA3 bitblits; like the BSP RGA3 register builder, the
  rewrite programs only the RGA3 scale direction and factor fields.
  AFBC is accepted through ``RGA_FBC_MODE`` for the RGA3 FBCD/FBCE format subset;
  tile8x8 is accepted through ``RGA_TILE_MODE`` for the BSP RGA3 semiplanar YUV
  tile-format subset in simple raster-to-tile and tile-to-raster bitblits.
  RFBC, AFBC32x8, packed-YUV FBC, compressed in-place alpha write-back,
  overlapping in-place blits, tile alpha/pattern/color-key, and in-place
  scaling/conversion/rotation remain unsupported.  Main request
  rotation/mirror flags are translated to RGA3 WIN0 rotate/mirror controls;
  unknown main rotate selector values fall back to no-op rotation like the BSP
  RGA2/RGA3 register builders while preserving rejection of per-channel rotate
  fields.
  BSP's RGA3 policy restriction for source YUV422 90/270-degree rotation is
  preserved so those jobs fall back to an eligible RGA2 core unless the request
  forced an RGA3-only core mask.  Per-channel rotate flags remain unsupported.
  Destination rectangle offsets are supported by biasing WR plane base
  addresses for 8-bit formats and raster unpacked semiplanar 10-bit YUV
  destinations; semiplanar YUV offsets must be chroma-aligned.
* RGA3 Porter-Duff alpha blend for the common A+B composition paths emitted by
  ``librga`` and ``ffmpeg-rockchip``.  Source/foreground is programmed through
  WIN1.  No-pattern A+B->B uses destination/background through WIN0 and writes
  back to the destination.  Pattern-backed A+B->C accepts ``bsfilter_flag`` with
  an imported pattern/source1 image, programs that image through WIN0, and writes
  WR to the destination.  Alpha flags ``0x19`` plus the non-premultiplied bit 9
  are accepted with per-pixel/global alpha selection.  This subset supports
  RGB-destination composition, including ffmpeg
  ``overlay_rkrga=format=<rgb>`` pipelines, no-pattern ``librga`` A+B->B
  updates for matching semiplanar YUV source/destination formats, and the
  default ffmpeg/RKMPP overlay path where an 8-bit or matching semiplanar
  10-bit YUV main/foreground image and RGB/RGBA pattern write a YUV destination
  through the per-window CSC policy generated by ``librga``.  Main-request
  rotation/mirror is applied to the foreground WIN1 path, including BSP-matched
  90-degree sizing for pattern and no-pattern A+B jobs.  Mixed-depth 8/10-bit
  YUV-destination alpha, converted no-pattern YUV A+B->B, and compressed
  in-place write-back remain unsupported.
* RGA3 normal-mode RGB color-key for the current ``librga`` ``imcolorkey``
  destination-update path.  The rewrite accepts the two-image raster RGB
  request shape emitted by ``librga``, routes it through the RGA3 overlap
  A+B->B topology, converts the 8-bit key min/max fields into the BSP RGA3
  overlap-key layout, and keeps the destination as the background image.
  Inverted color-key, pattern color-key, FBC/YUV/10-bit color-key, and
  hand-built color-key requests outside the normal ``librga`` alpha/zero-key
  mode remain unsupported.
* RGA2 solid color fill for the common ``librga`` ``imfill`` destination-only
  path.  The rewrite selects an RGA2-class RK3588 core, creates job-owned
  dma-buf mappings for that core when the imported handle was originally mapped
  elsewhere, and emits a minimal raster fill command for RGB-family
  destinations plus the 8-bit planar/semiplanar YUV destinations current
  ``librga`` documents for color fill.  Destination rectangle offsets are
  applied through the RGA2 destination address helper, including chroma-plane
  offsets and alignment checks for YUV fill.  YUV fill requires the
  RGB-to-YUV mode bits emitted by ``librga``.  This is deliberately an RGA2
  profile: the RK3588 forward-port RGA3 capability table does not advertise
  ``RGA_COLOR_FILL``, so requests forced to RGA3 core bits fail with
  ``-EOPNOTSUPP``.  ``imfillTaskArray()`` and ``imrectangleTask()``/array
  jobs are covered as serial multi-task RGA2 fill batches under one request
  completion/fence.  Pattern fill, alpha/ROP/color-key, rotation, tile/FBC,
  10-bit, and packed-YUV fill variants remain unsupported.
* RGA2 raster bitblit for the common upstream-consumer fallback formats that
  RGA3 does not cover: planar YUV420/YUV422, YCbCr400/gray, NV24/NV42-style
  YUV444 semiplanar, compact 10-bit semiplanar source, RGB555-family, and
  ARGB/ABGR output.  This path supports imported dma-buf backed
  source/destination images, single-handle and per-plane-handle images, linear
  raster scaling/conversion, BSP-compatible interpolation selector and downscale
  factor emission for current ``librga`` ``imresize()`` default/bicubic/linear
  requests, BSP-compatible ``rga_req.rotate_mode`` plus ``sina``/``cosa``
  rotation and mirror encoding, including the active-rectangle
  swap used by ``librga`` for 90/270-degree rotation, the BSP force-tile scale
  mode used by no-scale compact 10-bit and YUV444 semiplanar blits, and
  job-owned dma-buf mappings for the selected RGA2 core.  RGA2 destination
  full-CSC coefficients generated by ``librga`` for RGB-to-YUV conversion are
  accepted for this raster bitblit path and programmed through the
  BSP-compatible CSC register block.  Forced RGA2 core requests also accept the
  non-overlapping same-buffer, same-format, no-scale raster mirror copies emitted
  by current ``librga`` for ``immakeBorder()`` reflect/wrap left and right edge
  tasks.  RFBC64x4 source images emitted by ``ffmpeg-rockchip`` for Rockchip
  RFBC DRM frames are accepted for the BSP RGA2P YUV source-format subset and
  programmed through the RGA2 FBCIN register aliases while keeping the
  destination raster.  Pattern/alpha blend outside the alpha-bitmap subset
  below, color-key, ROP outside the ``imrop`` subset below, tile, AFBC32x8,
  RFBC destination, overlapping
  in-place blits, in-place scaling/conversion/rotation, incompact 10-bit input,
  10-bit output, OSD, pre-intr, and gauss/NN-quantize variants outside the
  subsets below remain unsupported.
* RGA2 in-place RGB mosaic for the current ``librga`` ``immosaic`` single-image
  path.  The rewrite accepts same-buffer, same-rectangle raster RGB bitblit
  requests with ``mosaic_info.enable`` and BSP mosaic modes ``0..4``.  Scaled,
  converted, rotated, YUV, pattern, OSD, gauss, and multi-rectangle mosaic
  variants remain unsupported.
* RGA2 RGB ROP bitblit for the current ``librga`` ``imrop`` single-task path.
  The rewrite accepts same-size, raster RGB source/destination requests encoded
  with ``alpha_rop_flag == 0x3``, ``alpha_rop_mode == 0x1``, and the public
  ``IM_ROP_AND``, ``IM_ROP_OR``, ``IM_ROP_NOT_DST``, ``IM_ROP_NOT_SRC``,
  ``IM_ROP_XOR``, or ``IM_ROP_NOT_XOR`` opcode values.  Pattern, mask, ROP4,
  scaled, converted, rotated, YUV, and in-place overlapping ROP variants remain
  unsupported.
* RGA2 RGB gaussian blur for the current ``librga`` ``imgaussianBlur`` and
  ``IM_GAUSS`` sample paths.  The rewrite copies the userspace-generated 3x3
  coefficient triplet into kernel-owned request/job storage during preparation,
  so async and acquire-fence-deferred jobs do not depend on the userspace
  coefficient pointer after ioctl return.  Only same-size, raster RGB
  source/destination bitblit requests with ``gauss_config.size == 3`` are
  accepted; scaled, converted, rotated, YUV, pattern, ROP, OSD, and
  multi-kernel gauss variants remain unsupported.
* RGA2 RGB NN quantize for current ``librga`` ``imquantize`` and
  ``imquantizeTask`` request shapes.  The rewrite accepts same-size, raster RGB
  source/destination bitblit requests encoded with ``alpha_rop_flag == BIT(8)``
  and programs the BSP-compatible 10-bit per-channel scale and offset fields
  carried in ``gr_color``.  Scaled, converted, rotated, YUV, in-place,
  alpha/ROP, OSD, gauss, and mixed-feature quantize variants remain
  unsupported.
* RGA2 RGB alpha-bitmap composite for the current ``librga``
  ``rga_alpha_rgba5551_demo``/``IM_ALPHA_BIT_MAP`` request shape.  The rewrite
  accepts separate raster RGB source/destination images plus an RGBA/BGRA/ARGB/
  ABGR5551 pattern image, the BSP-normalized ``DST_OVER | PRE_MUL`` alpha flag
  encoding, and the two alpha values carried by ``rgba5551_alpha``.  It programs
  RGA2 SRC1 format/address/stride state, the A1555 alpha remap enable,
  alpha0/alpha1 color registers, the BSP bitblt-mode selection for SRC1, and
  the BSP-compatible alpha control registers.  Other Porter-Duff modes,
  scaling, conversion, rotation, YUV/FBC, in-place,
  non-5551 pattern, OSD, gauss, ROP, and mixed-feature alpha-bitmap variants
  remain unsupported.
* RGA2 OSD alpha overlay for the current ``librga`` ``imosd`` and
  ``rga_alpha_osd_demo`` path.  The rewrite accepts the BSP shape emitted by
  userspace: same-buffer, same-rectangle RGB background source/destination,
  RGBA-family raster OSD image in SRC1, ``DST_OVER`` alpha controls, normal
  fixed-width OSD blocks, and the auto-invert/flag fields copied from
  ``im_osd_t``.  It programs SRC1 address/stride, the RGA2 bitblt-mode and OSD
  mode bits, BSP-compatible OSD control/flag/calibration registers, and the
  OSD-specific alpha premultiplication policy.  Scaled/rotated/converted, YUV,
  FBC/tile, RGBA2BPP/external-color, non-fixed-width, non-``DST_OVER``,
  ROP/color-key, mosaic, gauss, and mixed-feature OSD variants remain
  unsupported.
* RGA2 color palette for the current ``librga`` ``impalette`` sample path.
  The rewrite accepts the BSP two-command sequence emitted by userspace: first
  ``UPDATE_PALETTE_TABLE`` with the 16x16 RGBA8888 LUT image, then
  ``COLOR_PALETTE`` from a same-size BPP8/YCbCr400 raster source to an
  RGB-family raster destination.  Palette mode ``3`` and the little-endian
  color-palette source encoding used by current ``librga`` are programmed; the
  BSP userspace render mode ``6`` update request is remapped to RGA2 hardware
  render mode ``3`` when loading the LUT.  BPP1/2/4, scaled/rotated/converted,
  YUV destination, FBC/tile, pattern, alpha/ROP, OSD, and mixed-feature
  palette variants remain unsupported.
* Multi-task requests are accepted when every task matches a supported backend
  profile; tasks run serially under the request's single completion/fence.
  Mixed RGA2/RGA3 batches validate the complete request up front, then select
  an eligible backend for each task as it reaches the head of the serial batch.
  The current ``librga`` copy-splice task shape, two RGBA source tiles written
  into left/right rectangles of one larger RGBA destination, is covered by the
  RGA3 no-blend bitblit path and preserves each task's destination offset.
  The RGA3 no-blend emitted command includes overlap field and
  alpha/default-global-alpha controls.
* Legacy ``RGA_CACHE_FLUSH``, ``RGA_FLUSH``, ``RGA_GET_RESULT``, and
  ``RGA2_GET_RESULT`` as BSP-compatible no-ops.
* Optional ``ROCKCHIP_RGA_REWRITE_KUNIT_TEST`` coverage for rewrite-local ABI
  normalization helpers, including the RGA2 ``rotate_mode``/``sina``/``cosa``
  decoder, transformed destination-corner selection, color-fill core-mask
  dispatch, BSP request task-count limits and return codes, mixed RGA2/RGA3
  multi-task classification, request-id removal on terminal submit, RGA2 fill
  RGB/YUV destination-offset emission,
  YUV-fill chroma-alignment rejection, multi-fill task acceptance, and
  ``librga`` rectangle-task serial fill command emission, compact 10-bit RGA2
  source dispatch/emission including the no-scale force-tile mode, RGA2
  in-place RGB mosaic dispatch/emission, RGA2 RGB ROP dispatch/emission, RGA2
  RGB gauss coefficient lifetime and command emission, RGA2 RGB NN quantize
  dispatch/emission, RGA2 RGB alpha-bitmap SRC1/alpha emission, RGA2 OSD
  SRC1/alpha/control emission, RGA2 palette update and color-palette command
  emission,
  acquire-fence fd ownership merging, ffmpeg-facing RGA2 RFBC64x4 source
  profile selection and FBCIN command emission, IOMMU fault target matching,
  post-reset IOMMU refresh accounting, scheduler priority enqueue/aging,
  RGA3 normal RGB color-key dispatch/emission and inverted-mode rejection,
  RGA3 ``librga`` copy-splice multi-task destination-offset emission,
  RGA3 tile8x8 profile selection and stride emission, RGA3 pattern-backed
  8-bit ``librga`` alpha-YUV overlay emission, ``librga`` global-alpha
  register emission, 10-bit YUV alpha-overlay emission, and
  ffmpeg-facing RGA3 raster/FBC/alpha-overlay profile selection plus
  destination-offset command
  emission, source-crop command emission for RGA3 and RGA2, and semiplanar
  chroma-alignment rejection.

Recognized But Unsupported
--------------------------

* RGA2 hardware command generation outside the solid color fill, raster bitblit,
  and color-palette profiles above, including full-CSC outside raster bitblit.
* RGA3 pattern outside the supported alpha-overlay profile, color-key outside
  the normal RGB ``imcolorkey`` profile, converted no-pattern or mixed-depth
  8/10-bit YUV-destination alpha, per-channel rotation,
  RFBC/AFBC32x8, tile outside simple bitblits, compact 10-bit or AFBC
  destination offsets, virtual/physical-address, and non-bitblit operation
  modes.
Unsupported submit profiles return ``-EOPNOTSUPP`` after copying, validating,
preparing, queuing, dispatching, resolving imported buffers, allocating an owned
command buffer, and power-sequencing an owned job to the backend boundary.

Outside This Slice
------------------

* Physical address imports.
* Full RGA2/RGA3 command-register generation and policy selection.
* Full BSP timeout diagnostics and private-IOMMU recovery policy beyond the
  public post-reset domain refresh.
