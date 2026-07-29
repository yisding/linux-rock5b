Rockchip RGA Rewrite ABI Status
===============================

This rewrite registers ``/dev/rga`` when both ``ROCKCHIP_MULTI_RGA`` and
``VIDEO_ROCKCHIP_RGA`` are disabled and ``ROCKCHIP_RGA_REWRITE`` is enabled.

Implemented
-----------

* Legacy ``RGA_GET_VERSION`` and ``RGA2_GET_VERSION`` query paths.
  ``RGA2_GET_VERSION`` intentionally returns a positive success value after
  copying the version string, matching the BSP/librga observable contract.
* Modern ``RGA_IOC_GET_HW_VERSION`` and ``RGA_IOC_GET_DRVIER_VERSION``.
  Hardware version queries report the RK3588-compatible RGA2E
  ``3.2.63318`` and RGA3 ``3.0.76831`` tuples used by current ``librga``
  capability probing.  Probe powers each core long enough to decode its actual
  version register and admits only those exact implemented profiles; the
  generic BSP-compatible strings therefore cannot expose this RK3588-specific
  command/capability profile on another RGA revision.  Version ioctls are
  populated from the validated hardware value rather than the compatible's
  expected tuple.
* Native and compat ioctl entry points share the same fixed-width RGA parser,
  matching the BSP RGA3 compat entry behavior for current ``librga``.
* RGA2/RGA3 platform-driver binding for RK3588 BSP and mainline compatibles,
  with devm-managed MMIO, IRQ, clock, and reset discovery.  Every executable
  RGA node requires an IRQ and at least one discovered clock; a missing IRQ is
  propagated as a probe error, and an empty clock list fails with ``-EINVAL``
  instead of registering hardware that cannot complete jobs or allowing
  unclocked MMIO access.  Probe also verifies that the primary register resource
  covers every fixed direct access used by the backend: at least ``0x90`` bytes
  for RGA2 (through the full-CSC coefficient window) and ``0x44`` bytes for RGA3
  (through command state).  Missing or truncated windows fail before IRQ
  registration or job dispatch; the standard RK3588 BSP nodes provide
  ``0x1000`` bytes.  After runtime PM is enabled, probe reads RGA2 offset
  ``0x28`` or RGA3 offset ``0x18`` with the clocks enabled and fails with
  ``-ENODEV`` when the decoded version is not RGA2E ``3.2.63318`` or RGA3
  ``3.0.76831`` respectively.  When reset controls are described, probe first
  asserts the complete reset array, waits one microsecond, and deasserts it;
  failures abort probe rather than inheriting an asserted or stale bootloader
  state.
* ``RGA_IOC_IMPORT_BUFFER`` and ``RGA_IOC_RELEASE_BUFFER`` for dma-buf fd and
  userspace virtual-address imports owned by the open file session.  Imported
  dma-bufs are attached and mapped against a bound RGA hardware device through
  public dma-buf APIs.  A dma-buf value must be losslessly representable as a
  nonnegative ``int`` fd; upper-bit aliases are rejected with ``-EINVAL``
  instead of being truncated to another descriptor.  Virtual-address imports
  pin user pages, build sg_tables through ``sg_alloc_table_from_pages()``, map
  them with the public DMA API, and synchronize them around hardware execution
  for the common CPU-buffer ``librga`` sample paths.  Imports synthesized for
  legacy direct-address tasks size a compressed or tiled image from its own
  ``rd_mode`` layout (FBC header plus payload) rather than the raster
  width/height fallback, so the pinned range covers what materialization
  checks against.  The rewrite prefers an
  available RGA3 node for imports because that is the first executable hardware
  backend; RGA2 fallback jobs create job-owned remaps against the selected RGA2
  device.  A core quarantined after an unsuccessful recovery reset is no longer
  used as an import mapping device.
* ``RGA_IOC_REQUEST_CREATE``, ``RGA_IOC_REQUEST_CONFIG``, and
  ``RGA_IOC_REQUEST_CANCEL`` request lifetime management.
  ``RGA_IOC_REQUEST_CONFIG`` is a staging operation: it copies the userspace
  task array into the session request, resolves imported-buffer handles to
  mapped IOVAs, keeps the request id live, and does not submit hardware or
  export a release fence by itself.
  ``RGA_IOC_REQUEST_SUBMIT`` consumes the session request after cloning the
  prepared job resources, matching current ``librga`` task-job lifetime.
* Modern request task-array copy into session-owned request objects.  Handle
  backed image addresses are resolved to mapped IOVAs in the kernel-owned task
  copy, and configured requests hold references to their imported dma-bufs.
  After ``rga_request_check()`` accepts a modern request, config/submit
  preparation failures are normalized to the BSP ioctl wrapper's ``-EFAULT``.
* Build-time assertions for the fixed-width RGA version-query, image,
  import-buffer, buffer pool, user-request, and task ABI layouts plus legacy
  command constants and modern ioctl numbers and sizes.
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
* Rewrite-local ``dma_fence`` release fences and prepared-job completion path.
  Async jobs own a release fence internally and signal it with the eventual
  backend completion status; synchronous submits return that status directly.
  If userspace drops a pending async job before completion, cleanup signals the
  release fence with ``-EFAULT`` like the BSP request teardown path.  Every
  release fence carries its own ``dma_fence`` context: jobs complete on any of
  the four cores in any order, so a shared timeline would signal out of order
  and let a ``sync_file`` merge collapse distinct jobs into the highest seqno
  (the BSP's single-context allocation has exactly that flaw).  Each live
  fence also holds a module reference, because exported ``sync_file`` fds pin
  only the built-in sync core; module unload therefore fails with ``-EBUSY``
  instead of leaving fence ops and the fence lock dangling in freed module
  memory.
* Race-free release-fence fd publication.  Async submit reserves the descriptor
  and creates its ``sync_file``, copies the descriptor number to userspace, and
  installs the file only after that copy succeeds.  A usercopy fault drops the
  still-private ``sync_file`` and descriptor reservation, so rollback cannot
  close an unrelated file if another thread races fd close/reuse.  A legacy
  ``RGA_BLIT_ASYNC`` reply is copied from a pre-resolution snapshot of the
  caller's request with only ``out_fence_fd`` changed.  Handle-shaped addresses
  and userspace virtual addresses therefore round-trip unchanged; kernel-resolved
  IOVAs are neither disclosed nor raced against per-core rebasing by the
  dispatcher.
* File-close ownership for submitted jobs.  The open file session tracks every
  submitted sync/async job until completion.  ``release()`` marks the session
  closing, rejects newly tracked jobs, cancels unsignaled acquire-fence
  callbacks for jobs that have not reached hardware yet, completes those jobs
  and their release fences with ``-EFAULT``, and waits for acquire workers or
  successful multi-task IRQ handoffs that already committed to dispatch to
  publish their queue state before sweeping hardware.  It removes queued jobs
  owned by the closing session before they can reach hardware, resets active
  jobs owned by that session through the normal recovery path, then waits for
  the session job list to drain before dropping configured requests and
  imported buffers.
* Async pending-acquire handling for modern submit and legacy blit paths.  If
  an async job is blocked by an unsignaled acquire fence, the ioctl exports the
  release-fence fd, arms ``dma_fence`` callbacks, queues dispatch work when the
  acquire fences have signaled on the high-priority system workqueue like the
  forward port, and signals the release fence with the eventual backend result.
  Negative acquire-fence status is preserved as the submit/completion result;
  already-signaled success fences do not force the async pending path.  Abort
  and callback paths atomically claim each waiter, callback status is read with
  the fence lock already held, and completion waits for the pending-callback
  count (including the callback-arming sentinel) to drain.  Last-core removal
  racing callback setup therefore cannot release the shared work reference
  before later callbacks are armed.  Pending-acquire ownership is published as
  one session-locked state instead of being inferred from scheduler fields
  owned by other locks.  After callback arming, submit rechecks hardware
  availability and completes the exported fence with ``-ENODEV`` if the last
  core disappeared before that state was published.
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
  ``-ENODEV``, signals any exported async release fence with that result, and
  when the last RGA core disappears also completes async jobs still blocked on
  unsignaled acquire fences with ``-ENODEV``.  The removal transition is
  published under the same per-core job lock used by queue admission and the
  abort sweep, so a selected-core race is either rejected or swept.  Reprobe
  assigns the first vacant public core-mask bit for the hardware class rather
  than deriving a duplicate bit from the number of surviving peers.  Recovery
  reset failure similarly quarantines the affected core until reprobe: routing
  skips it, an admission race completes with ``-EIO``, jobs already queued on
  it are drained with ``-EIO``, and loss of the last usable core aborts async
  jobs still blocked on acquire fences.  The first quarantine transition is
  exposed as ``recovery_failure_count`` in debugfs.
* Backend-aware core selection for prepared jobs.  The scheduler checks the
  same supported-operation profile used by command generation, selects the
  least-loaded compatible RGA core for the current hardware-backed profile,
  rotates same-class equal-load ties across public core bits, and requires that
  core to match the device that owns the job's dma-buf mappings.
  Common bitblits that are accepted by both RGA3 and RGA2 participate in the
  same load-based choice so the RK3588 RGA2 core can take work when the RGA3
  cores are busier, matching the forward-port optional-core policy.
  BSP-compatible ``rga_req.core`` scheduler masks are honored for supported
  profiles: RGA3 core bits ``0x1``/``0x2`` select RGA3 cores and RGA2 bits
  ``0x4``/``0x8`` select RGA2 cores.  Unsupported profiles requested on a
  forced core still fail with the normal validation error, and a mask that
  selects only an absent hardware core is not silently rerouted to another
  present core.  Imported images are rebound to the selected core's DMA device
  at dispatch time, so direct
  ``wrapbuffer_fd()`` submissions can target a non-default RGA core.  Minimal
  debugfs counters report scheduled, dispatched, and hardware-started work per
  public core-mask bit.  ``import_count`` is a live imported-mapping gauge that
  remains nonzero while a configured request or job retains the mapping and
  returns to zero after its final reference is released.  Additional Route B
  counters report userptr IOMMU fallback attempts,
  successes, live mappings, and a force-remap knob under ``route_b/``.  The
  Route B counters are development evidence only, not ABI; they let validation
  distinguish a real scattered-userptr fallback pass from a workload that stayed
  on the normal DMA segment path.  Every userptr DMA mapping also owns independent
  cache-line boundary shadows for an unaligned first or last page.  Before device
  access the active bytes are copied into those pages; after device access only
  the active bytes are copied back.  This keeps cache maintenance and any DMA
  bounce handling away from adjacent userspace bytes, including per-core rebound
  mappings and Route B.  Cross-core task handoff synchronizes the mapping that
  actually ran before preparing the next core, so an inactive shadow cannot copy
  stale data over a result.  The ``shadow_*`` debugfs counters expose live head and
  tail views, setup failures, and copy byte totals for hardware attribution.
  RK3588 board validation can confirm load balancing,
  equal-load tie rotation, and forced-core routing without carrying the BSP
  debugger ABI.
* Runtime PM and clock-bulk sequencing around the backend dispatch boundary.
  Each dispatched job resumes the selected RGA core, enables discovered clocks,
  enters the backend, then disables clocks and drops runtime PM.
* Threaded IRQ registration and active-job completion scaffolding.  RK3588 RGA3
  registers its level IRQ with ``IRQF_SHARED`` because that line is shared with
  the external Rockchip IOMMU; the hard handler clears RGA status before waking
  the thread, so masking the complete shared line with ``IRQF_ONESHOT`` is
  unnecessary and would conflict with the IOMMU handler.  RGA2 retains
  ``IRQF_ONESHOT``.  The hard handler performs MMIO only while a job_lock-held
  regs-live count proves the core's clocks are up: a shared-line refire after
  power-off, or an interrupt in the probe window before the first power-on,
  returns ``IRQ_NONE`` instead of reading gated registers and stalling the
  bus.  Power-off drops that count under the same lock before gating clocks,
  so no handler is mid-read when they go down.  A backend can accept a job,
  leave it active, and have the
  IRQ thread complete it, signal fences, release runtime PM/clocks, and dispatch
  the next queued job.
  The top half decodes RGA2/RGA3 done and error interrupt status, clears handled
  bits, and propagates hardware error status to the job completion result.
  RGA2 config/parser error bit 25 is enabled and terminal, with its companion
  status word at ``0x024`` captured before clear and returned as ``-EACCES``.
  Raw interrupt/status values, a config-error counter, and decoded rectangle,
  overlay-bound, and odd-alignment causes are retained for diagnostics.
  Error bits take precedence when hardware reports DONE and ERROR together.
  Every error completion resets the core before clocks and runtime PM are
  released, matching the forward-port recovery requirement instead of sending
  the next queued job into error-contaminated state.
  Synchronous waiters acquire the completion flag published after the stored
  result, rather than racing plain ``done``/``result`` accesses.
* Master-mode RGA2/RGA3 hardware start helpers for generated command buffers.
  Accepted jobs enable done/error interrupts, program the selected core's command
  DMA address, start command execution, and arm a per-core timeout worker.  Match
  data carries the two RK3588 low-voltage quirks: RGA3 keeps its logic clock on
  through command issue, while RGA2E ``3.2.63318`` omits the unreliable
  frame-end automatic-reset bit.
* Per-core timeout completion for accepted hardware jobs.  Timed-out jobs read
  RGA2/RGA3 status registers, reset the selected core with the BSP-style
  soft-reset helper plus reset-controller fallback when present, ask the public
  IOMMU layer to flush the core's attached domain, complete with ``-EBUSY``,
  release runtime PM/clocks, and dispatch the next queued job.  If the status
  readback shows the blit actually finished -- its completion latched while
  the watchdog held the IRQ line masked -- the job completes with the normal
  interrupt-derived result instead of ``-EBUSY``, and the recovery reset still
  clears the undelivered latch.  Recovery
  fallback pulses the reset array through explicit
  assert/delay/deassert calls because Rockchip's reset provider does not
  implement the one-shot ``reset_control_reset()`` operation.  Timeout, fault,
  session-abort, and removal recovery disable the registered core IRQ and drain
  its hard handler before status readback, reset, or runtime suspend, preventing
  the top half from touching MMIO while recovery powers the core down.  Each
  timeout owns a reference to the exact job it watches; a stale running worker
  cannot claim a replacement slot, and replacement start rearms the shared
  delayed work even while that stale invocation exits.  IOMMU refresh and
  redispatch now require either a successful soft reset or a successful reset
  controller fallback.  If both are unavailable or fail, the core is
  quarantined rather than pretending recovery succeeded.  Its IRQ remains
  disabled so a stuck interrupt cannot re-enter the hard handler after clocks
  and runtime PM have been released.
* Public IOMMU fault callback registration for bound RGA cores.  A fault
  records ``iommu_fault_count`` in debugfs, logs the IOVA/status, marks the
  active job for immediate recovery through the same serialized reset path,
  refreshes the attached IOMMU domain after reset, completes the job with
  ``-EIO``, and signals any exported async release fence with that result.
  Fault recovery uses a dedicated work item carrying the activation generation
  observed under the active-job lock.  The worker only claims that generation
  and cancels the normal timeout after a match, so a delayed fault cannot abort
  a replacement job or consume its watchdog.
  Registration uses the Rockchip provider-local hook rather than the generic
  set-once DMA-domain handler; an attached-domain core fails probe if that hook
  is unavailable, while genuine no-IOMMU operation remains valid.  A provider
  source must match the exact RGA master or physical IOMMU inside a shared
  domain, so an unknown source is not redirected to the first peer.  Removal
  unlists the core, clears its provider callback independently of shared-domain
  peers, and waits for any provider IRQ callback already in flight before the
  driver or devm state can be released.
* Per-job DMA-coherent command-buffer allocation and lifetime, sized for the
  selected RGA2/RGA3 core and released through normal job teardown.  Probe
  applies the BSP address limits: RGA2 uses 32-bit streaming and coherent DMA
  masks, while RGA3 uses a 40-bit streaming mask and a 32-bit coherent mask.
  The complete command-buffer DMA span is checked before its 32-bit base is
  programmed, so a high allocation is rejected instead of being truncated.
* RGA3 command-buffer generation path for validated raster and AFBC16x16
  bitblits.  The no-blend profile normally maps source to WIN0 and destination
  to WR; non-overlapping same-buffer, same-format, no-scale raster mirror copies
  use the BSP-compatible overlap route with source on WIN1, destination/background
  on WIN0, WR on the destination, and the destination rectangle carried in
  ``OVLP_OFF``.  This covers the current ``librga`` ``immakeBorder()``
  reflect/wrap top/bottom source-to-destination tasks and reflect/wrap
  same-destination side-edge tasks, plus the current ``librga`` DRM-fourcc
  ``rga_copy_drm_fourcc_demo`` ``DRM_FORMAT_ABGR8888``/modifier-zero path after
  userspace maps it to ``RK_FORMAT_RGBA_8888``.  The path requires MMU-mapped
  source/destination images: modern handle requests use the session import
  table, while legacy direct-buffer requests create job-owned temporary imports
  from dma-buf fds or user virtual addresses.  It rejects color-key and
  unsupported pattern operations, and supports the 8-bit RGB/YUV plus
  semiplanar 10-bit YUV formats exposed by common ``librga`` and
  ``ffmpeg-rockchip`` blit/scale/convert users.
  For RASTER and TILE 10-bit semiplanar images ``vir_w`` carries the row stride in
  **bytes** (the legacy BSP contract "width_stride equals byte_stride" that
  ``librga``'s legacy blit path and the JeffyCN GStreamer plugin rely on):
  compact NV15/NV20 pack 10 bits per pixel, incompact P010/P210 carry 16-bit
  containers, and the register writers use that byte stride directly (with the
  TILE writer's additional eight-lines-per-block factor).  The active window's
  rows, in bytes, must fit inside ``vir_w``.  Compressed FBC modes keep the
  pixel-count ``vir_w`` convention because their payload stride math scales
  ``vir_w`` itself; ``act_w``/``x_offset`` are always pixels.
  For RGB-to-YUV BT.709 limited conversion, RGA3 accepts the exact
  ``full_csc`` enable flag current ``librga`` also supplies for its RGA2E
  workaround, ignores the coefficient block, and emits RGA3's native direct
  CSC mode.  Other flags, standards, and custom full-CSC requests remain
  rejected on RGA3.
  This also covers the current JeffyCN ``gstreamer-rockchip`` MPP plugin's
  legacy ``c_RkRgaBlit()`` conversions: malloc-backed RGB-family input to
  dma-buf NV12 for encoder preprocessing, MPP-frame NV12/NV21-style dma-buf
  input to RGB-family output with optional rotation, and planar I420/YV12-style
  fallback through RGA2 when RGA3 cannot accept the format.  The KUnit matrix
  pins the userspace-visible GStreamer set for BGR16/RGB/BGR/RGBA/BGRA/RGBx/
  BGRx to NV12, NV16/NV61 to NV12, NV12/NV21/NV16/NV61 plus compact 10-bit
  NV12/NV16 decoder output to RGB-family formats, compact NV12_10LE40/
  NV16_10LE40 decoder output to scaled 8-bit NV12/NV16, and planar I420/YV12
  round-trips through the RGA2 fallback.
  Explicit interpolation selectors from current ``librga`` resize calls are
  accepted on native RGA3 bitblits; like the BSP RGA3 register builder, the
  rewrite programs only the RGA3 scale direction and factor fields.  The
  scheduler applies the BSP RGA3 capability floor of 68x2 active pixels and the
  per-axis 1/8x..8x scale range before selecting a core.  Smaller windows and
  wider scale ratios drop the RGA3 eligibility bit and can fall back to RGA2E's
  2x2 and 1/16x..16x ranges instead of being programmed outside the declared
  RGA3 capability.
  AFBC is accepted through ``RGA_FBC_MODE`` for the RGA3 FBCD/FBCE format
  subset, including current ``librga`` ``rga_copy_fbc_demo`` AFBC16x16
  raster-to-FBC and FBC-to-raster YUV420SP copies, current ffmpeg AFBC
  RGB-family and compact 10-bit YUV input/output, plus ffmpeg/RKMPP AFBC
  source crop-top offsets carried as active-window y-offsets; tile8x8 is
  accepted through ``RGA_TILE_MODE`` for the BSP RGA3 semiplanar YUV
  tile-format subset in simple raster-to-tile, tile-to-raster, and
  tile-to-tile bitblits.  AFBC destination offsets use the same overlap-offset
  programming as the BSP, preserving the base FBCE addresses for no-blend and
  alpha writeback paths.
  RFBC, AFBC32x8, packed-YUV FBC, compressed in-place alpha write-back,
  overlapping in-place blits, tile alpha/pattern/color-key, and in-place
  scaling/conversion/rotation remain unsupported.  Main request
  rotation/mirror flags are translated to RGA3 WIN0 rotate/mirror controls,
  including current ``librga`` ``imrotate()``, ``imflip()``, and combined
  rotate-plus-mirror RGB blits; unknown main rotate selector values fall back
  to no-op rotation like the BSP RGA2/RGA3 register builders while preserving
  rejection of source/destination per-channel rotate fields and pattern-channel
  rotate fields on pattern-blend requests.  ``librga`` submits the 90/270-degree
  destination window pre-swapped (``act_w`` carries the canvas height) while
  the vir strides stay in canvas orientation; the emitters and scale checks
  consume that wire form directly, and the rectangle, tile, and
  subsampling-alignment validators check the canvas orientation, mirroring
  the vendor's un-swap plus its explicit act-versus-vir rotate exemption, so
  genuine portrait rotate requests are not rejected.
  BSP's RGA3 policy restriction for source YUV422 90/270-degree rotation is
  preserved so those jobs fall back to an eligible RGA2 core unless the request
  forced an RGA3-only core mask.  Per-channel rotate flags remain unsupported.
  Destination rectangle offsets are supported by biasing WR plane base
  addresses for 8-bit formats and raster compact/unpacked semiplanar 10-bit YUV
  destinations; this covers current ``librga`` ``imtranslate()`` RGB requests
  and centered RGB rotate requests with explicit destination rectangles, plus
  the ffmpeg ``overlay_rkrga`` pre-processing pass that copies a smaller RGB
  overlay into an offset full-frame pattern image before alpha blending.
  Semiplanar YUV offsets must be chroma-aligned.
* RGA3 Porter-Duff alpha blend for the common A+B composition paths emitted by
  ``librga`` and ``ffmpeg-rockchip``.  Source/foreground is programmed through
  WIN1.  No-pattern A+B->B uses destination/background through WIN0 and writes
  back to the destination.  Pattern-backed A+B->C accepts ``bsfilter_flag`` with
  an imported pattern/source1 image, programs that image through WIN0, and writes
  WR to the destination.  Alpha flags ``0x19`` plus the non-premultiplied bit 9
  are accepted with per-pixel/global alpha selection.  This subset supports
  RGB-destination composition, including ffmpeg
  ``overlay_rkrga=format=<rgb>`` pipelines, current ``librga``
  ``rga_alpha_3channel_demo``/``imcomposite()`` RGB SRC_OVER requests,
  no-pattern ``librga`` A+B->B updates for semiplanar 8/10-bit YUV
  source/destination formats, including mixed-depth, UV-order, and 4:2:0/4:2:2
  conversions, the current RK3588 ``im2d_slt`` three-channel RGB/RGBA
  foreground plus RGB/RGBA background scale-up into a semiplanar YUV
  destination with pattern/background rectangle offsets, and the default
  ffmpeg/RKMPP overlay path where a semiplanar 8/10-bit YUV main/foreground
  image and RGB/RGBA pattern write a YUV destination, including mixed-depth
  source/writeback and AFBC16x16 output when requested, through the per-window
  CSC and compact-10-bit policy generated by ``librga``.
  Main-request rotation/mirror is applied to the
  foreground WIN1 path, including BSP-matched 90-degree sizing for pattern and
  no-pattern A+B jobs.  The no-pattern A+B->B rotate path follows the same
  pre-swapped wire convention as simple bitblits; the pattern-blend rotate
  path keeps its historical canvas-form model, and its wire-form semantics
  remain unresolved against the vendor.  The current public
  ``librga`` Porter-Duff modes SRC, DST, SRC_OVER, DST_OVER, SRC_IN, DST_IN,
  SRC_OUT, DST_OUT, SRC_ATOP, DST_ATOP, XOR, and CLEAR are accepted; other
  unlisted blend modes remain unsupported.  Compressed in-place write-back
  remains unsupported.
* RGA3 normal/inverted-selector RGB color-key for the current ``librga``
  ``imcolorkey`` destination-update path.  The rewrite accepts the two-image
  raster RGB request shape emitted by ``librga``, routes it through the RGA3
  overlap A+B->B topology, converts the 8-bit key min/max fields into the BSP
  RGA3 overlap-key layout, and keeps the destination as the background image.
  Like the BSP RGA3 register builder, the normal and inverted userspace
  selectors program the same overlap top-key command stream.  Pattern
  color-key, FBC/YUV/10-bit color-key, and hand-built color-key requests
  outside the normal ``librga`` alpha/zero-key mode remain unsupported.
* RGA2 RGBA-family color-key for the current ``librga``
  ``rga_alpha_colorkey_demo``/``imcolorkey`` forced-core profile.  The rewrite
  accepts separate raster alpha-carrying RGB source/destination images encoded
  with the normal ``librga`` alpha/zero-key flags, PD SRC blend mode, and
  ``src_trans_mode`` values ``0x1e`` (normal) or ``0x1f`` (inverted).  It
  programs the BSP RGA2 source-transparency bits, key min/max registers, and
  non-premultiplied per-pixel alpha controls.  Pattern color-key, YUV/FBC/tile,
  non-alpha source/destination formats, and other hand-built source-transparency
  requests remain unsupported.
* RGA2 solid color fill for the common ``librga`` ``imfill`` destination-only
  path.  The rewrite selects an RGA2-class RK3588 core, creates job-owned
  dma-buf mappings for that core when the imported handle was originally mapped
  elsewhere, and emits a minimal raster fill command for RGB-family
  destinations plus the 8-bit planar/semiplanar YUV destinations current
  ``librga`` documents for color fill, including packed YUV422 destination
  formats.  Destination rectangle offsets are
  applied through the RGA2 destination address helper, including chroma-plane
  offsets and alignment checks for YUV fill.  YUV fill requires the
  RGB-to-YUV mode bits emitted by ``librga``.  This is deliberately an RGA2
  profile: the RK3588 forward-port RGA3 capability table does not advertise
  ``RGA_COLOR_FILL``, so requests forced to RGA3 core bits fail with
  ``-EOPNOTSUPP``.  ``imfillTaskArray()`` and ``imrectangleTask()``/array
  jobs are covered as serial multi-task RGA2 fill batches under one request
  completion/fence.  Packed-YUV420 destination fill variants use the same RGA2
  write-only destination format path as packed-YUV422.  Pattern fill,
  alpha/ROP/color-key, rotation, tile/FBC, and 10-bit fill variants remain
  unsupported.
* RGA2 ``IM_PRE_INTR`` line-interrupt programming for already-supported RGA2
  profiles.  The rewrite accepts current ``librga`` ``improcess`` requests that
  set ``pre_intr_info``, programs the BSP read-line threshold, write-line
  start/step, and read-hold mode fields, enables the RGA2 line read/write
  interrupt bits, and clears line-only interrupts without treating them as job
  completion.  RGA3 pre-intr remains unsupported because the checked BSP RGA3
  register path exposes no matching pre-intr programming sequence.
* RGA2 raster bitblit for the common upstream-consumer fallback formats that
  RGA3 does not cover: planar YUV420/YUV422, YCbCr400/gray, NV24/NV42-style
  YUV444 semiplanar, compact 10-bit semiplanar source, RGB555-family, and
  ARGB/ABGR output.  This path supports imported dma-buf backed
  source/destination images, single-handle and per-plane-handle images, linear
  raster scaling/conversion, BSP-compatible interpolation selector and downscale
  factor emission for current ``librga`` ``imresize()`` default/bicubic/linear
  requests, BSP-compatible ``rga_req.rotate_mode`` plus ``sina``/``cosa``
  rotation and mirror encoding, including the active-rectangle
  swap used by ``librga`` for 90/270-degree rotation -- the destination
  rectangle is validated in canvas orientation after un-swapping, mirroring
  the vendor's explicit act-versus-vir rotate exemption -- the BSP force-tile
  scale mode used by no-scale compact 10-bit and YUV444 semiplanar blits, and
  job-owned dma-buf mappings for the selected RGA2 core.  RGA2 destination
  full-CSC coefficients generated by ``librga`` for RGB-to-YUV conversion,
  including current ``imcvtcolor()`` BT.709 limited-range requests selected by
  ``imsetColorSpace()`` and current ``rga_cvtcolor_gray256_demo`` RGBA to
  ``YCbCr_400`` BT.601 full-range requests, are accepted for this raster
  bitblit path and
  programmed through the BSP-compatible CSC register block.  The current
  ``librga`` UV-downsampling helper that wraps NV16/NV12 Y and UV regions as
  tall ``YCbCr_400`` images is covered by the RGA2 gray/Y400 resize path,
  including the UV-region source and destination y-offsets.  Forced RGA2 core
  requests also cover the current ``librga`` RGB-to-Y4/Y8 dither output path
  for same-size, unrotated compact-CSC bitblits, including BSP-compatible
  Y4/Y8 destination format bits, dither mode, and Y4 map LUT command
  registers.  The same RGA2 path accepts the non-overlapping same-buffer,
  same-format, no-scale raster mirror copies emitted by current ``librga`` for
  ``immakeBorder()``
  reflect/wrap left and right edge tasks.  RFBC64x4 source images emitted by
  legacy ``ffmpeg-rockchip`` for Rockchip RFBC DRM frames, and AFBC32x8
  split-mode source images from ``librga`` DRM/gralloc paths, are RGA2-Pro
  compatibility modes rather than RK3588 required ABI.  Public RK3588 material
  identifies the SoC as one RGA2-Enhance core plus two RGA3 cores, the current
  ffmpeg target removed RFBC modifiers, and the rewrite now rejects these
  source-only FBCIN profiles with ``-EOPNOTSUPP`` instead of carrying an
  executable RGA2-Pro path.
  Pattern/alpha blend outside the alpha-bitmap and color-key subsets below,
  Y4/Y8 full-CSC dither output outside the current RGB-to-YUV ``librga`` path,
  color-key outside the RGBA ``imcolorkey`` profile above, ROP outside the
  ``imrop`` subset below, tile, YUV AFBC32x8, AFBC32x8 destination, RFBC
  destination, overlapping
  in-place blits, in-place scaling/conversion/rotation, incompact 10-bit input,
  10-bit output, Y4/Y8 full-CSC output, OSD, and gauss/NN-quantize
  variants outside the subsets below remain unsupported.
* RGA2 in-place RGB mosaic for the current ``librga`` ``immosaic`` single-image
  path.  The rewrite accepts same-buffer, same-rectangle raster RGB bitblit
  requests with ``mosaic_info.enable`` and BSP mosaic modes ``0..4``.
  ``immosaicTaskArray()`` jobs are covered as serial multi-task RGA2 mosaic
  batches under one request completion/fence.  The plain ``immosaicArray()``
  helper submits one supported mosaic task per rectangle and merges async
  fences in current ``librga`` userspace.  Scaled, converted, rotated, YUV,
  pattern, OSD, gauss, and mixed-feature mosaic variants remain unsupported.
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
  multi-kernel gauss variants remain unsupported.  When the caller does not
  set ``feature.global_alpha_en``, the gauss path defaults the source global
  alpha to opaque ``0xff`` like the vendor, so an unset flag cannot zero the
  blurred output's alpha channel.
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
  mode bits, BSP-compatible OSD control/flag/calibration registers, optional
  external normal/invert colors from ``IM_OSD_COLOR_EXTERNAL``, and the
  OSD-specific alpha premultiplication policy.  Scaled/rotated/converted, YUV,
  FBC/tile, RGBA2BPP, non-fixed-width, non-``DST_OVER``, ROP/color-key,
  mosaic, gauss, and mixed-feature OSD variants remain
  unsupported.
* RGA2 color palette for the current ``librga`` ``impalette`` sample path.
  The rewrite accepts the BSP two-command sequence emitted by userspace: first
  ``UPDATE_PALETTE_TABLE`` with the 16x16 RGBA8888 LUT image, then
  ``COLOR_PALETTE`` from a same-size BPP1/2/4/8 or ``YCbCr_400`` raster source
  to an RGB-family raster destination.  Palette modes ``0..3`` and the
  little-endian color-palette source encoding used by current ``librga`` are
  programmed; the BSP userspace render mode ``6`` update request is remapped
  to RGA2 hardware render mode ``3`` when loading the LUT.  Scaled/rotated/converted,
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
* Legacy ``RGA_CACHE_FLUSH``, ``RGA_FLUSH``, ``RGA2_FLUSH``,
  ``RGA_GET_RESULT``, and ``RGA2_GET_RESULT`` as BSP-compatible no-ops.
* Optional ``ROCKCHIP_RGA_REWRITE_KUNIT_TEST`` coverage for rewrite-local ABI
  normalization helpers, including required clock-count, per-generation MMIO
  aperture, and RGA2/RGA3 IRQ-flag selection validation, the RGA2
  ``rotate_mode``/``sina``/``cosa``
  decoder, transformed destination-corner selection, color-fill core-mask
  dispatch, BSP request task-count limits and return codes, legacy/modern
  version-query strings, positive success returns, and absent-RGA2 failure,
  mixed RGA2/RGA3 multi-task classification with per-task core-mask forcing and
  RGA3-to-RGA2 core handoff/requeue selection, request-id removal on terminal
  submit, request config reconfiguration resource replacement,
  request config reconfiguration acquire-fence replacement,
  request config reconfiguration gauss-coefficient replacement and job cloning,
  request-config ioctl staging with kernel-owned acquire-fd close and no
  release-fence export,
  request cancel and file-close cleanup of configured imports/fences,
  file-close cleanup of async jobs pending on acquire fences and jobs already
  queued on hardware,
  request create/cancel ioctl id allocation, usercopy, and miss handling,
  legacy ``RGA_BLIT_ASYNC`` acquire-fence ioctls that copy a release-fence fd
  back through ``rga_req.out_fence_fd`` before deferred dispatch while
  preserving the caller's pre-resolution addresses,
  modern request-submit async acquire-fence ioctls that return a release-fence
  fd before deferred dispatch and signal it with the eventual backend result,
  release-fence descriptor reservation staying invisible until publication and
  safe abort of an uninstalled reservation,
  import-buffer ioctl upper-bit dma-buf fd alias rejection,
  physical-address rejection, and malformed-pool returns,
  release-buffer ioctl handle removal and malformed-pool returns,
  ``librga`` virtual-address import sizing and physical import rejection,
  direct physical-address submit rejection after temporary import rollback,
  RGA2 fill RGB/YUV destination-offset emission,
  packed YUV422 fill destination format/offset emission,
  YUV-fill chroma-alignment rejection, packed-YUV420 fill emission,
  multi-fill task acceptance, and
  ``librga`` rectangle-task serial fill command emission, compact 10-bit RGA2
  source dispatch/emission including the no-scale force-tile mode, RGA2
  BPP import stride sizing, BPP1/2/4/8 color-palette source-mode selection,
  stride/offset emission, and update-palette mode validation, RGA2
  in-place RGB mosaic dispatch/emission, RGA2 RGB ROP dispatch/emission, RGA2
  multi-rectangle ``librga`` mosaic-task serial emission,
  RGB gauss coefficient lifetime and command emission, RGA2 RGB NN quantize
  dispatch/emission, RGA2 RGB alpha-bitmap SRC1/alpha emission, RGA2 OSD
  SRC1/alpha/control/external-flag/invert-calibration emission, RGA2 palette
  update and color-palette command
  emission,
  acquire-fence fd ownership merging, acquire-fence pending/success/error
  status propagation, lock-safe async acquire-callback error completion and
  abort-during-callback-arming lifetime,
  queued hardware-removal abort completion and release-fence signaling,
  last-hardware pending-acquire abort completion and release-fence signaling,
  selected-core removal race completion and release-fence signaling,
  RGA2-Pro RFBC64x4/AFBC32x8 source profile rejection, exact shared-domain
  IOMMU fault-source matching, exact IOMMU-fault activation attribution and
  replacement-watchdog preservation, exact timeout-job targeting and
  stale-worker replacement rearming, hot-reprobe core-slot preservation,
  post-reset IOMMU refresh accounting, 32-bit IOVA/command-buffer span bounds,
  RK3588 RGA2/RGA3 low-voltage start-word quirks, RGA2 config-error mask/result
  precedence and parse-status coverage, userptr shadow boundary/layout overflow
  cases and active-byte copy integrity,
  scheduler priority enqueue/aging,
  scheduler core-counter and per-core timing mapping,
  RGA3 tile8x8 raster/tile round-trip, byte-stride layout for 10-bit TILE
  surfaces, and tile-to-tile command emission,
  RGA2 ``librga`` full-CSC RGB-to-YUV dispatch/emission,
  RGA3 BT.709-limited direct-CSC dispatch/emission with the narrow ignored
  ``librga`` full-CSC compatibility flag,
  RGA2 ``librga`` gray256 RGB-to-Y400 color-conversion dispatch/emission,
  RGA2 ``librga`` Y400 UV-downsampling resize dispatch/emission,
  RGA2 ``librga`` Y4/Y8 compact/full-CSC dither-output dispatch/emission,
  RGA2 OSD external-color dispatch/emission, RGA2 pre-intr register packing
  and acceptance on supported fill/bitblit profiles,
  RGA3 normal/inverted RGB color-key dispatch/emission and
  invalid-selector rejection,
  RGA3 ``librga`` DRM-fourcc ABGR8888-to-RGBA normal raster copy emission,
  RGA3 ``librga`` copy-splice multi-task destination-offset emission,
  RGA3 multi-task command-buffer rebuild between alpha and plain copy tasks,
  RGA3 ``librga`` RGB translate and ffmpeg overlay-preprocess destination-offset emission,
  RGA3 ``librga`` RGB rotate, flip/mirror, and combined rotate/mirror
  emission, RGA3 active-window and 1/8x..8x scale capability selection with
  RGA2 fallback, RGA3 ``librga`` centered RGB rotate destination-offset emission,
  JeffyCN ``gstreamer-rockchip`` legacy ``c_RkRgaBlit()`` RGB-to-NV12,
  rotated NV12-to-RGB, and planar-I420 RGA2 fallback profile selection and
  command emission,
  RGA3 ``librga`` ``immakeBorder()`` reflect/wrap top/bottom command emission,
  RGA3 ``librga`` ``immakeBorder()`` reflect/wrap left/right side-edge command
  emission, RGA3 ``librga`` AFBC16x16 copy profile selection and FBCD/FBCE command
  emission including RGB-family/compact-10-bit read/writeback, ffmpeg
  AFBC-to-AFBC filter copies, and source active-offset handling, RGA3 tile8x8
  profile selection and stride emission, RGA3
  pattern-backed 8/10-bit mixed-depth ``librga`` alpha-YUV overlay,
  no-pattern semiplanar YUV alpha conversion, and AFBC writeback emission,
  ``librga`` global-alpha register emission, RGA3 ``librga`` three-channel
  RGB alpha-composite routing/emission, current ``librga`` Porter-Duff blend-mode factor mapping
  including CLEAR emission, 10-bit YUV alpha-overlay and AFBC writeback emission, and
  ffmpeg-facing RGA3 raster/FBC/alpha-overlay profile selection, P210-style
  unpacked 10-bit 4:2:2 stride/offset emission, plus
  destination-offset command
  emission, source-crop command emission for RGA3 and RGA2, and semiplanar
  chroma-alignment rejection.

Recognized But Unsupported
--------------------------

* RGA2 hardware command generation outside the explicitly enumerated solid
  color fill, raster bitblit, color-palette, and update-palette profiles above,
  including full-CSC outside raster bitblit.
* RGA3 pattern outside the supported alpha-overlay profile, color-key outside
  the RGB ``imcolorkey`` profile, per-channel rotation, RFBC/AFBC32x8 source
  and destination modes, tile outside simple bitblits, physical-address
  channels, and non-bitblit operation modes.
* Legacy ``RGA_IMPORT_DMA`` and ``RGA_RELEASE_DMA`` command numbers are
  recognized but return ``-EINVAL``; modern buffer import/release uses the
  implemented ``RGA_IOC_IMPORT_BUFFER``/``RGA_IOC_RELEASE_BUFFER`` path.

Unsupported task profiles are rejected by the scheduler's complete-task
validation before a job is enqueued, dispatched, power-sequenced, or given a
hardware command buffer.  Legacy blit ioctls preserve the internal
``-EOPNOTSUPP`` result.  After ``rga_request_check()`` accepts a modern request,
``RGA_IOC_REQUEST_CONFIG`` normalizes preparation failures to the BSP wrapper's
``-EFAULT``; it may stage an unsupported profile because backend validation is a
submit-time operation.  ``RGA_IOC_REQUEST_SUBMIT`` likewise normalizes its
preparation and profile-validation failures to ``-EFAULT``.  Any imported
resources, tracked job state, private async fence reservation, and copied task
storage are rolled back before return.

Outside This Slice
------------------

* The remainder of the BSP RGA2/RGA3 command-register matrix and capability
  policy beyond the explicitly enumerated profiles above.
* BSP-private page-table inspection and extended timeout register dumps beyond
  the public fault callback, reset, quarantine, and post-reset domain refresh
  implemented here.
