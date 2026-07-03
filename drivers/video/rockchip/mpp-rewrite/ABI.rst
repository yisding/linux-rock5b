Rockchip MPP Rewrite ABI Status
================================

This rewrite registers ``/dev/mpp_service`` when ``ROCKCHIP_MPP_SERVICE`` is
disabled and ``ROCKCHIP_MPP_REWRITE`` is enabled.

Implemented
-----------

* ``MPP_IOC_CFG_V1`` message parsing, including multi-message batches.  Staged
  register jobs in one batch are all submitted before poll requests are
  processed in message order, matching the forward-port trigger-then-wait
  ordering.
* Native and compat ioctls share the same BSP-style ``MPP_IOC_CFG_V1`` parser.
  The externally parsed message is the fixed-width 24-byte V1 record with a
  64-bit userspace data pointer; the driver converts it to a native internal
  ``struct mpp_request`` only after copying the V1 record from userspace.
* Build-time assertions guard the fixed-width V1 message and batch-entry
  layouts plus the ``MPP_IOC_CFG_V1`` type, number, direction, and size.
* ``MPP_CMD_SET_SESSION_FD`` session switching, restricted to other
  ``/dev/mpp_service`` file descriptors.  Switching sessions closes the
  current staged job and starts a distinct job for subsequent register/poll
  messages, including when a later batch entry switches back to an earlier
  session.  Invalid descriptors are reported in the batch entry's
  ``mpp_bat_msg.ret`` field as ``-EBADF``, and ``MPP_BAT_MSG_DONE`` entries
  are consumed as no-op markers.
* RK3588 BSP-style RKVENC2/RKVDEC2 platform-device binding with devm-managed
  MMIO, IRQ, clock, and reset discovery.
* ``MPP_CMD_QUERY_HW_SUPPORT`` from bound RK3588 MPP hardware cores.
* ``MPP_CMD_QUERY_HW_ID`` returns the register-0 hardware id captured from the
  first bound RK3588 RKVENC2/RKVDEC2 core whose referenced CCU coordinator is
  online, matching the forward port's userspace-visible HAL-selection
  contract for enabled RK3588 nodes.
* ``MPP_CMD_QUERY_CMD_SUPPORT`` group boundary queries.
* Minimal ``/proc/mpp_service/supports-cmd`` and
  ``/proc/mpp_service/support_cmd`` discovery markers so current
  ``mpp-rockchip`` enables command probing.  These procfs files are read-only
  compatibility markers, not BSP debug/control ABI.  The marker contents use
  the BSP-style labelled hexadecimal command table and include the current
  RK3588 command boundaries used by userspace capability probing.
* ``MPP_CMD_INIT_CLIENT_TYPE`` for detected classes.
* ``MPP_CMD_INIT_DRIVER_DATA`` as a validated no-op.
* ``MPP_CMD_INIT_TRANS_TABLE`` storage using the BSP-compatible ``u16`` table
  element width.
* ``MPP_CMD_RESET_SESSION`` import cleanup.
* ``MPP_CMD_TRANS_FD_TO_IOVA`` and ``MPP_CMD_RELEASE_FD`` through public
  dma-buf attach/map/unmap APIs against a bound client hardware device.
  Explicit translation maps against the default matching core for compatibility.
  The session import cache is keyed by both fd and DMA device, and
  ``RELEASE_FD`` drops all cached mappings for that fd.
* ``MPP_CMD_SEND_CODEC_INFO`` as validated per-session codec-info storage.
* ``MPP_CMD_SET_ERR_REF_HACK`` as validated copy-in/discard for current
  ``mpp-rockchip`` VDPU382 H.264 capability probing.
* Kernel-owned per-session job staging for register write/read, offset, RCB,
  and poll messages.  The rewrite preserves userspace message order and copies
  write-like payloads before returning to userspace.
* Flat register-image materialization for ``SET_REG_WRITE`` requests, bounded
  readback descriptor retention for ``SET_REG_READ`` requests, and validated
  register-offset tuple storage for ``SET_REG_ADDR_OFFSET`` requests.
* Register-image fd-to-IOVA translation using either the session-provided
  translation table or the RK3588 RKVENC2/RKVDEC2 default translation tables.
  Translated register jobs map dma-bufs against the selected hardware core's
  DMA device.  The ``REG_NO_OFFSET``/``REG_OFFSET_ALONE`` flag aliases preserve
  the BSP split between plain fd register values and separate offset records.
  Translated jobs retain references to every imported dma-buf mapping they use
  so ``MPP_CMD_RELEASE_FD``, ``RESET_SESSION``, and close do not tear mappings
  down while a prepared job still owns them.
* Session-owned active-job bookkeeping with explicit submit/complete/abort
  transitions.  ``RESET_SESSION`` and file close abort active rewrite jobs
  before releasing imports.
  KUnit coverage checks that reset/close-style abort removes both queued and
  session-active jobs from the session-visible list, drops queued counters and
  scheduler ownership, reports ``-ECANCELED`` internally, and leaves
  subsequent nonblocking poll with the BSP-compatible empty-session ``-EIO``.
* Prepared register jobs select and hold a counted reference to an online
  RK3588 hardware core whose ``rockchip,ccu`` phandle, when present, resolves
  to a bound online CCU coordinator.  Kernel-translated jobs choose the
  least-loaded matching core, prefer an idle core, and rotate equal-load
  automatic selections by core id so sequential submit streams do not stick to
  the first bound core.  Jobs flagged ``MPP_FLAGS_REG_FD_NO_TRANS`` stay on the
  default matching core so explicit ``TRANS_FD_TO_IOVA`` results remain
  device-consistent.  Core removal unlists the device and waits for prepared
  job references to drain before devm-managed resources are released.
* CCU coordinator removal makes dependent cores unavailable for new prepared
  jobs and completes already queued or active dependent-core jobs with
  ``-ENODEV``.
* Pending-job poll/abort lifetime infrastructure: accepted jobs remain on the
  session pending list until ``POLL_HW_FINISH`` consumes the completed result
  or reset/close aborts the session.
* First RK3588 RKVENC2/RKVDEC2 hardware execution slice: one active job per
  bound core, runtime-PM power-domain resume, bulk clock enable, range-checked
  MMIO writes from the original ``SET_REG_WRITE`` spans, start-register
  deferral, IRQ-driven completion, retained ``SET_REG_READ`` register readback,
  BSP-style interrupt-status readback override, and decoder RLC decoded-length
  adjustment.
* Contended submits queue internally instead of sleeping in the ioctl submit
  path.  A rewrite-local dispatcher feeds queued jobs to idle selected cores,
  preserves the existing session poll ordering, keeps BSP-style submit-time
  validation for invalid register/readback/start descriptors, and reports
  backend start failures through the normal completion path.  Minimal debugfs
  counters report scheduled, dispatched, and hardware-started work per
  RKVENC2/RKVDEC2 core id so RK3588 board validation can confirm multicore
  load balancing, DCHS routing, and decoder CCU distribution without carrying
  the BSP debugger ABI.
* RK3588 RKVENC2 DCHS dual-core hand-shake setup for queued multicore encoder
  jobs.  The rewrite mirrors the BSP-visible policy of always enabling TX,
  allocating per-active-core TX ids, linking RX to a same-session producer
  when the original ids match, allocating an RX id only while RX remains
  enabled, disabling RX when no producer is active, and
  clearing the DCHS slot on normal completion, submit failure, timeout, reset,
  close, or device removal.
* Per-core start/abort/timeout/completion serialization.  ``RESET_SESSION``,
  close, timeout recovery, and device removal cannot reset or power down a core
  while the queued dispatcher is still programming the accepted job.
* Per-core software timeout completion for active RKVENC2/RKVDEC2 jobs using
  the same 500 ms timeout window as the BSP-derived forward port.  A timed-out
  job is removed from the active hardware slot, the core reset line is pulsed
  when available, runtime PM/clocks are released, ``POLL_HW_FINISH`` wakes, and
  the job returns ``-ETIMEDOUT``.  For hard-CCU RKVDEC2 jobs, timeout recovery
  first force-stops and resets the coordinator, preserves table-complete jobs
  that raced the timeout by reading back their CCU link tables, then relinks
  unfinished tables, resets matched active dependent cores, and resends those
  jobs through the coordinator.  After reset and before resend, each matched
  active core asks the public IOMMU layer to flush the core's attached domain;
  in the target Rockchip IOMMU driver this maps to the hardware ``ZAP_CACHE``
  command.  If an unfinished job can no longer be matched to its active core
  slot, recovery falls back to opportunistically aborting unfinished active
  dependent cores without waiting on possibly running timeout workers.
* Public IOMMU fault callback registration for bound MPP cores.  A fault
  records ``iommu_fault_count`` in debugfs, logs the IOVA/status, marks the
  active job for immediate recovery through the same serialized reset path,
  and completes the job with ``-EIO``.  This deliberately avoids private
  Rockchip IOMMU state and page-table walking.
* RK3588 RKVDEC2 performance-selector readbacks for ``SET_REG_READ`` requests
  in the BSP ``0x20000`` selector window.  The rewrite validates this as a
  64-word logical side buffer, programs the selector register, reads the three
  selector-value registers, and copies results back through the normal
  readback path without treating the selector window as real MMIO.
* RK3588 VDPU383/RKVDEC2 link-MMIO direct start and IRQ handling.  Decoder
  cores with a validated ``link`` MMIO window follow the BSP VDPU383 start path:
  program link timeout/IP-enable registers, clear stale link IRQ/status
  latches, start through the link enable register, and complete from the
  link IRQ/status registers.  Cores without that window retain the direct
  decoder start-register fallback.
* RK3588 RKVDEC2 CCU-mode task register preparation for cores that declare a
  ``rockchip,ccu`` phandle.  The rewrite patches the BSP session/film index,
  disables the multicore PU/COLMV offset timeout reset bit, and writes the same
  20/50/100 ms timeout-threshold register values that the BSP selects from
  retained decoder width/height/bitdepth codec info when those task register
  words are present in the submitted image.
* RK3588 VDPU383/RKVDEC2 hard-CCU link-table capability data, backing
  allocation, table materialization, and readback copying.  Decoder cores with
  a ``link`` MMIO window and ``rockchip,ccu`` phandle validate the BSP VDPU383
  link register offsets and allocate DMA-coherent per-task link-table nodes
  sized from ``rockchip,task-capacity`` using public DMA APIs.  Accepted
  decoder jobs reserve one node from a per-core bitmap, stage the BSP
  write/readback register partitions into that job-owned table node, hold a
  counted reference to the referenced CCU coordinator, and release both on
  completion or abort.  The per-core active table list is relinked whenever a
  node is staged or released, so the tail points at an unused node and earlier
  active tables point at the next active table, matching the BSP add-mode queue
  invariant.  Jobs also record BSP-shaped hard-CCU submit descriptor values for
  all online cores behind the selected coordinator: core-work mask, table
  address, link-mode word,
  autogate/work/cfg-done bits, and the link IRQ CCU-mode bit.  When the CCU is
  idle, the rewrite powers the selected core's online peer decoder cores,
  programs each powered core's link window for CCU-work mode, mirrors the BSP
  fixed-RCB setup by writing the device-tree ``rockchip,rcb-info`` register
  layout once per core and setting the fixed-RCB link latch, starts that job
  through the hard-CCU config path, tracks the started job on the coordinator's
  running list, relinks that list as a coordinator-wide table chain, scans it on
  completion, and copies BSP table readback partitions back into the normal
  register image with the VDPU383 link-mode status word from the table
  interrupt-status word.  Hard-CCU register start/stop writes are serialized on
  the coordinator, and completion releases stop CCU work only after the
  coordinator running list drains.  If the job that powered peer decoder cores
  completes while the coordinator still owns queued hard-CCU work, those peer
  core references move to the next listed CCU job and are released only when the
  chain drains or aborts.  If the CCU is already active and the rewrite has a
  tracked coordinator chain, additional jobs are appended with the BSP
  ``ADD_MODE`` link-mode bit; if the hardware is busy without tracked jobs, the
  job keeps the direct link-MMIO fallback.  Completion matching for the selected
  active job checks that job's own CCU table, even if an earlier listed table
  has already finished.  After completing the IRQ core's own job, the thread
  scans the same coordinator for other table-complete jobs and completes them
  only when it can claim the exact active job from that job's owning core; busy
  cores are left for their own IRQ or timeout path.  If a hard-CCU completion
  table reports a VDPU383 error bit, the rewrite preserves the readback status
  for userspace but force-stops and resets the coordinator, resets the reporting
  core, drains any other table-complete jobs it can claim, relinks unfinished
  tables, resets active dependent cores that can still be matched to their
  owning hardware slots, and resends them.  If resend cannot safely restart
  every unfinished job, recovery aborts the remaining active dependents to
  contain the failed chain.
* ``MPP_CMD_POLL_HW_IRQ`` for RK3588 RKVENC2 encoder slice result streaming.
  The rewrite advertises the forward-port ``POLL_BUTT`` command boundary,
  detects slice mode from the submitted RKVENC2 register image
  (slice-length FIFO plus slice split enabled), queues slice-length words from
  slice/done IRQs in a job-owned FIFO, copies up to userspace ``count_max``
  entries into the flexible poll buffer, and falls through to normal
  completion/readback handling on the final slice.  Non-split jobs polled
  through ``POLL_HW_IRQ`` use the same full-frame completion path as
  ``POLL_HW_FINISH``.
* ``MPP_CMD_SET_RCB_INFO`` for RK3588 RKVENC2/RKVDEC2 jobs.  The rewrite stores
  BSP-compatible ``(register index, size)`` descriptors per session, snapshots
  them into each job, allocates per-core coherent scratch memory using the
  device-tree ``rockchip,rcb-iova`` size, and patches matching registers after
  fd-to-IOVA translation and offset handling.  Out-of-range register indices
  are ignored without consuming scratch space.  Decoder RCB obeys the BSP
  ``rockchip,rcb-min-width`` gate using retained ``SEND_CODEC_INFO`` width.
  This uses the public DMA API rather than BSP fixed-IOVA SRAM reservation.
* Refcounted batch/session/hardware job ownership so a concurrent poll, reset,
  close, or IRQ thread does not free a job while another owner is still using it.
* Optional ``ROCKCHIP_MPP_REWRITE_KUNIT_TEST`` coverage for rewrite-local ABI
  parser helpers, including command range classification, command group
  boundary queries, fixed-width V1/``mpp_bat_msg`` ABI layout, V1-to-native
  request conversion, payload-copy classification, ``SEND_CODEC_INFO`` storage
  and trailing-byte tolerance, register-span overflow
  checks, BSP VDPU383 link IRQ decoding, link-table
  layout/materialization/readback/ownership/relinking, CCU-reference lifetime,
  hard-CCU running-list table-chain relinking/scanning/active matching and
  active-job out-of-order matching/drain detection, hard-CCU done-table
  detection, peer-core power ownership transfer, release-time peer-core
  ownership transfer, unfinished-chain relinking for hard-CCU resend
  preparation, unfinished-job collection for hard-CCU resend, active-slot retry
  preservation and IOMMU refresh accounting, cross-core CCU completion
  claiming, hard-CCU table-status readback,
  hard-CCU idle/add-mode descriptor values, hard-CCU all-core work-mask
  selection, fixed-RCB link-latch programming, decoder RCB min-width gating,
  MPP core-counter routing, IOMMU fault target matching,
  ``INIT_TRANS_TABLE`` ``u16`` storage and boundary behavior,
  ``RELEASE_FD`` import-cache sweeping across all DMA-device mappings for one
  fd while preserving other fd imports,
  ``SET_REG_ADDR_OFFSET`` tuple staging/apply behavior and cap handling,
  procfs support-command table coverage,
  RKVENC2 DCHS tx/rx id remapping and release, independent-core DCHS id
  capacity, ``POLL_HW_IRQ``
  flexible-buffer sizing, RKVENC2 slice-mode detection and slice FIFO
  overflow/final-slice reporting, ``POLL_HW_FINISH`` nonblocking pending-job
  ``-EAGAIN`` without consuming the active job, and ``SET_SESSION_FD``
  invalid-fd status, done-marker handling, and batch job splitting.

Recognized But Unsupported
--------------------------

* No required RK3588 MPP userspace command is intentionally left in the
  recognized-but-unsupported bucket.

Outside This Slice
------------------

* Board-level stress validation of hard-CCU timeout/error recovery under real
  IOMMU faults, runtime suspend, and decoder reset races on RK3588 hardware.
* MPP fence export/import semantics.  The observed RK3588 MPP UAPI in
  ``include/uapi/linux/rk-mpp.h`` exposes no fence command or fence flags, and
  the BSP-derived 6.18 driver does not provide a sync-file fence path in the
  checked MPP service sources; this remains a future compatibility item only if
  current userspace evidence appears.
* ``MPP_IOC_CFG_V2``; the BSP-derived 6.18 driver also rejects this in the
  observed path.
