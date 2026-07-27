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
  are consumed as no-op markers.  The dormant libmpp batch-server wait layout
  is recognized narrowly as repeated ``SET_SESSION_FD`` +
  ``POLL_HW_FINISH|POLL_NON_BLOCK|LAST_MSG`` pairs and rejected with
  ``-EOPNOTSUPP`` instead of extending the BSP ABI with multiple
  ``LAST_MSG`` groups in one ioctl.
* RK3588 BSP-style RKVENC2/RKVDEC2 platform-device binding with devm-managed
  MMIO, IRQ, clock, and reset discovery.
  Hardware-backed encoder/decoder cores and the decoder CCU require a nonempty
  clock list and fail probe with ``-EINVAL`` when none is discovered.  The
  zero-clock encoder CCU remains valid because that node is only a virtual
  coordinator and has no MMIO hardware to power.
  Those hardware-backed matches also require the complete RK3588 primary
  register window (RKVENC2 ``0x6000``, RKVDEC2 ``0x5a0``, decoder CCU
  ``0x100``); a missing or truncated resource fails probe with ``-EINVAL``
  before hardware-ID or job register access and before support is advertised.
  The RKVDEC2 span includes the VDPU381 cache/max-outstanding-read registers
  through offset ``0x59c``; board descriptions expose a page-safe ``0x600``
  function aperture ending immediately before the MMU at function offset
  ``0x600``.  Direct jobs and idle HARD-CCU chain starts therefore cannot
  silently skip cache setup because a truncated ``0x400`` resource passed
  probe.
  After powering each core, probe also requires the register-0 identity used by
  current libmpp for the selected backend: VEPU58x ``0x50603312`` for RKVENC2
  and VDPU38x ``0x53813f05`` for RKVDEC2.  A zero, missing, or different
  identity fails probe with ``-ENODEV`` before IRQ/fault registration or
  support advertisement, so generic compatible strings cannot silently select
  the RK3588 VEPU580/VDPU381 register and link-table profiles on another
  hardware revision.
  Encoder and decoder cores require an available ``rockchip,ccu`` phandle of
  the corresponding encoder/decoder CCU compatible; decoder cores additionally
  require a mirrored one-hot ``rockchip,core-mask`` in the RK3588 CCU's low and
  high two-bit fields (``0x00010001`` or ``0x00020002``).  Missing, disabled,
  cross-type, zero, multi-core, unmirrored, or out-of-field topology fails probe
  instead of silently exposing the rewrite's non-CCU fallback behavior or
  programming unrelated CCU bits.
  Public core IDs are allocated from the first vacant bounded slot when a DT
  alias is absent, while out-of-range or already-used aliases fail probe.
  Decoder cores behind one CCU must also have nonoverlapping core masks.  This
  preserves scheduler counters, encoder DCHS ownership, and decoder CCU work
  bits across removal/reprobe instead of deriving a duplicate ID from the
  number of surviving devices.
  Device-tree ``rockchip,normal-rates`` entries that match the clock list are
  applied through the public clock framework before hardware clocks are enabled,
  preserving the BSP-visible fixed-rate performance setup without depending on
  private devfreq or Rockchip OPP internals.
  Probe failure and platform removal cancel pending autosuspend work and
  synchronously suspend an idle device before runtime PM is disabled, so the
  hardware-ID probe or the last completed job cannot leave its generic power
  domain active after unbind.
* ``MPP_CMD_QUERY_HW_SUPPORT`` from bound RK3588 MPP hardware cores.
* ``MPP_CMD_QUERY_HW_ID`` returns the validated register-0 hardware id captured
  from the first bound RK3588 RKVENC2/RKVDEC2 core whose referenced CCU
  coordinator is online, matching the forward port's userspace-visible
  HAL-selection contract for enabled RK3588 nodes.
* ``MPP_CMD_QUERY_CMD_SUPPORT`` group boundary queries.
* Per-message flag validation before any payload, session-switch, or hardware
  processing.  Unknown bits and flags used outside their command class return
  ``-EINVAL`` instead of being silently accumulated into a job.  The normal
  libmpp transport, register/offset, and nonblocking-poll combinations remain
  accepted.
* Minimal ``/proc/mpp_service/supports-cmd`` and
  ``/proc/mpp_service/support_cmd`` discovery markers so current
  ``mpp-rockchip`` enables command probing.  These procfs files are read-only
  compatibility markers, not BSP debug/control ABI.  The marker contents use
  the BSP-style labelled hexadecimal command table and include the current
  RK3588 command boundaries used by userspace capability probing.
* ``MPP_CMD_INIT_CLIENT_TYPE`` for detected classes.  The first successful
  command binds the session type; repeating that type is idempotent, while an
  attempted encoder/decoder rebind returns ``-EBUSY`` instead of changing the
  interpretation of already staged or active register work.
* ``MPP_CMD_INIT_DRIVER_DATA`` as a validated no-op.
* ``MPP_CMD_INIT_TRANS_TABLE`` storage using the BSP-compatible ``u16`` table
  element width.  Each staged job snapshots the current table, client type,
  codec information, and inherited RCB descriptors, and successful state
  controls split later messages into a new staged job.  A later control message
  therefore cannot retroactively change an earlier job in the same or a racing
  ioctl.  On the fixed RK3588 backends, a session-provided table may add
  address registers but cannot replace the built-in VEPU580/VDPU381 table;
  known DMA registers are always translated or validated, so an incomplete
  custom table cannot turn an address register into an unchecked literal IOVA.
* ``MPP_CMD_RESET_SESSION`` staged/active job and import cleanup.  Reset
  advances a per-session epoch before aborting active work, drops earlier
  staged jobs for that session from the current multi-message ioctl, and rejects
  racing stale jobs with ``-ECANCELED``.  Active-list and scheduler-queue
  ownership are published together under the session lock, so reset cannot
  miss a job in the gap between those two lists.
* ``MPP_CMD_TRANS_FD_TO_IOVA`` and ``MPP_CMD_RELEASE_FD`` through public
  dma-buf attach/map/unmap APIs against a bound client hardware device.
  The first explicit translation selects the default matching core for
  compatibility, maps every fd in that command against the same DMA device,
  and pins that device as the session's explicit-IOVA affinity.  Later
  translations use the same device instead of silently moving outstanding
  IOVAs to another IOMMU domain.  Translation and release are serialized so a
  racing ``RELEASE_FD`` cannot unmap an IOVA before its translation is returned.
  The translated IOVA array is copied back after dropping the service-wide
  hardware mutex, so a faulting userspace destination cannot stall unrelated
  codec admission or IRQ completion; the narrower explicit-map lock retains the
  release exclusion.  The affinity is cleared when ``RELEASE_FD`` empties the
  import cache, or by reset/close.
  Both commands require a nonempty, bounded payload containing an exact array
  of 32-bit fd elements; partial trailing bytes are rejected rather than
  silently ignored.
  The session import cache is keyed by fd, DMA device, and the dma-buf object
  resolved while the ioctl holds its own reference.  Reusing an integer fd for
  another dma-buf therefore cannot return the old mapping; obsolete mappings
  for that fd are unlinked while any in-flight job references remain valid.
  ``RELEASE_FD`` drops all cached mappings for that fd.  Because RK3588 codec
  registers expose one 32-bit base IOVA rather than a scatterlist, each mapped
  dma-buf must cover its full size as one byte-contiguous DMA span within the
  32-bit aperture; discontiguous, truncated, zero-length, or overflowing spans
  fail before submission.  A ``MPP_FLAGS_REG_FD_NO_TRANS`` job is accepted only
  after explicit translation has pinned one DMA device; every nonzero address
  named by the custom-plus-built-in table must fall inside one of that session's
  dma-buf mappings on the selected device.  The job holds those mapping
  references through completion, so concurrent release cannot invalidate a
  literal IOVA already admitted to hardware.
* Every MPP hardware node is configured with the 32-bit streaming and coherent
  DMA masks required by the codec register ABI.  Driver-owned coherent RCB and
  hard-CCU link-table allocations are also checked across their complete span,
  so an address above 4 GiB cannot be truncated into a 32-bit hardware register.
* ``MPP_CMD_SEND_CODEC_INFO`` as validated per-session codec-info storage.
* ``MPP_CMD_SET_ERR_REF_HACK`` as validated copy-in/discard for current
  ``mpp-rockchip`` VDPU382 H.264 capability probing.
* Kernel-owned per-session job staging for register write/read, offset, RCB,
  and poll messages.  The rewrite preserves userspace message order and copies
  write-like payloads before returning to userspace.
* Flat register-image materialization for ``SET_REG_WRITE`` requests, bounded
  readback descriptor retention for ``SET_REG_READ`` requests, and validated
  register-offset tuple storage for ``SET_REG_ADDR_OFFSET`` requests.
* Register-image fd-to-IOVA translation using the RK3588
  RKVENC2/RKVDEC2 default translation tables plus any session-provided entries.
  Translated register jobs map dma-bufs against the selected hardware core's
  DMA device.  The ``REG_NO_OFFSET``/``REG_OFFSET_ALONE`` flag aliases preserve
  the BSP split between plain fd register values and separate offset records.
  The rewrite retains each translated register's dma-buf provenance while
  applying ``SET_REG_ADDR_OFFSET``.  Embedded and separate offsets are combined
  with checked arithmetic and rejected when their cumulative value is outside
  the mapped dma-buf or makes the 32-bit register IOVA overflow, instead of
  allowing hardware to fault past its mapping.  Offset tuples for literal
  non-fd registers retain the BSP's additive behavior, but reject cumulative
  32-bit wrap instead of programming the wrapped register value.
  Both kernel-translated and validated explicit-IOVA jobs retain references to
  every imported dma-buf mapping they use so ``MPP_CMD_RELEASE_FD``,
  ``RESET_SESSION``, and close do not tear mappings down while a prepared job
  still owns them.  Import creation checks the same session epoch before cache
  lookup and again before insertion, preventing a pre-reset translation from
  repopulating the emptied cache after reset.
* Session-owned active-job bookkeeping with explicit submit/complete/abort
  transitions.  ``RESET_SESSION`` and file close abort active rewrite jobs
  before releasing imports; their epoch change also invalidates work staged by
  a concurrent ioctl before it can select, translate, or enter the scheduler.
  KUnit coverage checks that reset/close-style abort removes both queued and
  session-active jobs from the session-visible list, drops queued counters and
  scheduler ownership, reports ``-ECANCELED`` internally, and leaves
  subsequent nonblocking poll with the BSP-compatible empty-session ``-EIO``.
* Prepared register jobs select and hold a counted reference to an online
  RK3588 hardware core whose ``rockchip,ccu`` phandle, when present, resolves
  to a bound online CCU coordinator.  Kernel-translated jobs choose the
  least-loaded matching core, prefer an idle core, and rotate equal-load
  automatic selections by core id so sequential submit streams do not stick to
  the first bound core.  Once a session has returned explicit IOVAs, jobs
  flagged ``MPP_FLAGS_REG_FD_NO_TRANS`` select the online hardware instance
  whose DMA device owns those mappings.  They return ``-ENODEV`` instead of
  falling back to a different IOMMU device while that instance is removed; a
  rebind of the same platform device restores selection without invalidating
  the cached mappings.  Before any explicit translation, literal no-translate
  jobs fail with ``-ENODEV`` instead of choosing an unproven default DMA
  domain.  KUnit coverage checks missing affinity, online affinity, offline
  rejection, same-device rebind, in-range mapping retention, and rejection at
  both ends of a mapped span.  It also checks that CCU
  coordinator teardown completes queued and active jobs on dependent decoder
  cores with ``-ENODEV`` while dropping scheduler and hardware ownership.
  Removing a core behind a CCU
  coordinator also quiesces hard-CCU work and aborts queued/active jobs on the
  remaining cluster members, so a decoder job cannot retain an obsolete core
  mask and an encoder job cannot retain a vanished DCHS peer.  Core removal
  unlists the device and waits for prepared job references to drain before
  devm-managed resources are released.  SOFT- and HARD-CCU starts revalidate
  the coordinator, selected core, and cancellation state after acquiring the
  coordinator run lock, closing the race where removal could quiesce the CCU
  after the earlier availability check but before the final MMIO start writes.
* CCU coordinator removal makes dependent cores unavailable for new prepared
  jobs and completes already queued or active dependent-core jobs with
  ``-ENODEV``.
* Pending-job poll/abort lifetime infrastructure: accepted jobs remain on the
  session pending list until ``POLL_HW_FINISH`` consumes the completed result
  or reset/close aborts the session.  Active-job hardware-pointer pin/detach is
  serialized by the session lock: abort acquires its own hardware reference
  before touching timeout/run state, while completion clears the job-owned
  pointer before dropping that reference.  Concurrent platform removal
  therefore cannot pass its final hardware-release wait and free the devm
  object between abort's pointer load and use.
* First RK3588 RKVENC2/RKVDEC2 hardware execution slice: one active job per
  bound core, runtime-PM power-domain resume, bulk clock enable, range-checked
  MMIO writes from the original ``SET_REG_WRITE`` spans, start-register
  deferral, IRQ-driven completion, retained ``SET_REG_READ`` register readback,
  BSP-style interrupt-status readback override, and decoder RLC decoded-length
  adjustment.  The RLC delta and ten-bit ABI scaling are evaluated as unsigned
  32-bit arithmetic, preserving the BSP bit pattern while avoiding an undefined
  signed left shift if an error/wrap status reports an address below the stream
  start.
* Contended submits queue internally instead of sleeping in the ioctl submit
  path.  A rewrite-local dispatcher feeds queued jobs to idle selected cores,
  preserves the existing session poll ordering, keeps BSP-style submit-time
  validation for invalid register/readback/start descriptors, and reports
  backend start failures through the normal completion path.  Debugfs counters
  report scheduled, dispatched, hardware-started, completed, failed, IRQ, and
  spurious-IRQ work, including per-core scheduling and timing.  This lets
  RK3588 board validation confirm multicore load balancing, DCHS routing, and
  decoder CCU distribution without carrying the BSP debugger ABI.
* RK3588 RKVENC2 DCHS dual-core hand-shake setup for queued multicore encoder
  jobs.  The rewrite mirrors the BSP-visible policy of always enabling TX,
  allocating per-active-core TX ids, linking RX to a same-session producer
  when the original ids match, allocating an RX id only while RX remains
  enabled, disabling RX when no producer is active, and
  clearing the DCHS slot on normal completion, submit failure, timeout, reset,
  close, or device removal.  An occupied core slot or exhausted TX-id set is a
  submission error; hardware is not started with an untracked handshake.
* Per-core start/abort/timeout/completion serialization.  ``RESET_SESSION``,
  close, timeout recovery, and device removal cannot reset or power down a core
  while the queued dispatcher is still programming the accepted job.  Recovery
  disables the registered core IRQ and drains its hard handler before claiming,
  resetting, or runtime-suspending the active slot; a threaded completion that
  was already in flight finishes before recovery selects its target.  Each
  watchdog also owns a reference to its exact job.  If cancellation races a
  worker that has already started, that worker cannot claim a replacement slot,
  and replacement start uses ``mod_delayed_work()`` to guarantee a fresh
  watchdog even while the stale invocation exits.  Aborting a queued job whose
  selected core is actually running another session's job restores that
  active job's watchdog after the ownership check.  Encoder submit-failure
  teardown disables and drains the hard IRQ before gating clocks or dropping
  the active-slot reference.
  Encoder and direct decoder error IRQs pulse the reporting core's reset line
  before releasing runtime PM.  SOFT-CCU decoder error/timeout/IOMMU recovery
  additionally follows the BSP per-core coordinator sequence: force that core
  idle, pulse its reset, clear its CCU error latch, and reconnect it without
  disturbing a healthy peer core.  Reset assertion/deassertion errors are
  ratelimited in the kernel log; an otherwise successful error-IRQ completion
  reports that reset error instead of silently claiming containment succeeded.
  SOFT-CCU latch clear/reconnect is skipped when the core reset fails.  Any
  reset assertion/deassertion failure permanently quarantines the affected
  core until reprobe, including reset deassertion that fails during runtime
  power-up: selection/admission and the dispatcher skip it, already queued jobs
  complete with ``-EIO``, and its IRQ remains disabled so a stuck line cannot
  access powered-off MMIO.  A coordinator reset failure applies the same
  quarantine and queue drain to every dependent decoder core.  Debugfs exposes
  the state per hardware row and counts first failures in
  ``recovery_failure_count``.
  If reset cannot prove that a failed DMA-capable core stopped, the rewrite
  permanently attaches its complete IOMMU group to a probe-time preallocated
  empty paging domain after barring admission for every bound group member.
  A coordinator without an IOMMU group instead requires its generic power
  domain to report physically off.  Either proof is terminal for the current
  bound instance: power-on is rejected.  An isolated domain is
  deliberately retained rather than reopening DMA during module teardown, so
  later probe of that group also fails until reboot.  Probe fails up front when
  an attached IOMMU group cannot provide this isolation mechanism.
  Interrupt status remains available through normal register readback.
* Per-core software timeout completion for active RKVENC2/RKVDEC2 jobs using
  the same 500 ms timeout window as the BSP-derived forward port.  A timed-out
  job is removed from the active hardware slot, the core reset line is pulsed
  when available, runtime PM/clocks are released, ``POLL_HW_FINISH`` wakes, and
  the job returns ``-ETIMEDOUT``; SOFT-CCU decoder timeout recovery also clears
  and reconnects the reporting core in the coordinator as described above.
  For hard-CCU RKVDEC2 jobs, timeout recovery
  first force-stops and resets the coordinator, preserves table-complete jobs
  that raced the timeout by reading back their CCU link tables, then relinks
  unfinished tables, resets matched active dependent cores, and resends those
  jobs through the coordinator.  After reset and before resend, each matched
  active core asks the public IOMMU layer to flush the core's attached domain;
  in the target Rockchip IOMMU driver this maps to the hardware ``ZAP_CACHE``
  command.  Coordinator or dependent-core reset failure prevents resend and
  falls into the existing fail-closed dependent-job abort path.  If an
  unfinished job can no longer be matched to its active core
  slot, recovery falls back to opportunistically aborting unfinished active
  dependent cores without waiting on possibly running timeout workers.  If a
  collected resend job races completion or removal, both its reset-preparation
  and restart pass snapshot a counted hardware reference under the session
  lock before touching IRQ or run-lock state; a retained job reference alone
  therefore cannot outlive and dereference a detached core.  If a
  peer is still inside its serialized start/completion section, recovery pins
  that exact active job before trying the run lock and queues an immediate
  deferred abort only if that same job still owns the slot.  The worker blocks
  on the peer run lock, claims that exact target, and only then cancels its
  timeout.  Lock contention therefore cannot defer containment to the ordinary
  500 ms timeout, abort a replacement job, or remove the replacement's
  watchdog when a stale target is discarded.
* Public IOMMU fault callback registration for bound MPP cores.  A fault
  records ``iommu_fault_count`` in debugfs, logs the IOVA/status, marks the
  active job for immediate recovery through the same serialized reset path,
  and completes the job with ``-EIO``.  This deliberately avoids private
  Rockchip IOMMU state and page-table walking.  Provider-local callbacks are
  owned and cleared per physical IOMMU, even when decoder cores share one DMA
  domain; a callback carrying a source device must match that exact controller
  or master before recovery is accepted.  In HARD-CCU mode, the physical source
  may be a peer of the software-selected job owner.  The rewrite therefore
  reads the source link block's BSP ``CFG_ADDR``/error-descriptor register,
  matches that descriptor IOVA to the active coordinator job, and schedules
  recovery on the owning software slot.  The software-started ownership flag is
  published with one-copy semantics and ordered before the ``CFG_DONE``
  doorbell, so a fault raised immediately by descriptor start cannot miss that
  owner.  Fault recovery has its own work item and snapshots the owning slot's
  activation generation under the active-job lock.  The worker only claims
  that exact activation and cancels its ordinary watchdog after the match, so
  delayed fault work cannot complete a replacement job or consume the
  replacement's timeout.  If the descriptor cannot be matched, any active job
  in that HARD coordinator is used to enter the existing
  force-stop/coordinator-wide abort path instead of scheduling an empty peer
  slot and silently waiting for the normal timeout.  If
  the public provider hook is unavailable, fault reporting remains disabled:
  the legacy domain callback is set-once, cannot be safely unregistered from a
  default DMA domain that outlives a loadable MPP driver, and is not used as a
  fallback.  If an IOMMU domain is attached but the public provider hook is
  unavailable, core probe fails instead of running without the intended fault
  recovery path; cores operating without an IOMMU domain do not require it.
* RK3588 RKVDEC2 performance-selector readbacks for ``SET_REG_READ`` requests
  in the BSP ``0x20000`` selector window.  The rewrite validates this as a
  64-word logical side buffer, programs the selector register, reads the three
  selector-value registers, and copies results back through the normal
  readback path without treating the selector window as real MMIO.
* RK3588 VDPU381/RKVDEC2 link-MMIO coordination.  SOFT-CCU jobs program the
  core/CCU work-mode bits at link offset ``0x00`` and complete from the decoder
  core interrupt-status register.  HARD-CCU jobs use the raw link interrupt at
  that same offset, preserve its work-mode bits while acknowledging the IRQ,
  and retain the decoder core status as the userspace-visible interrupt word.
  Non-CCU jobs start and complete through the decoder core registers directly.
* RK3588 RKVDEC2 CCU-mode task register preparation for cores that declare a
  ``rockchip,ccu`` phandle.  The rewrite patches the BSP session/film index,
  disables the multicore PU/COLMV offset timeout reset bit, and writes the same
  20/50/100 ms timeout-threshold register values that the BSP selects from
  retained decoder width/height/bitdepth codec info when those task register
  words are present in the submitted image.  If untrusted codec dimensions
  overflow the pixel-count calculation, the rewrite selects the longest
  100 ms threshold instead of wrapping into a shorter watchdog interval.
* RK3588 RKVDEC2 CCU mode selection from the coordinator node's
  ``rockchip,ccu-mode`` property.  Missing or invalid values fall back to the
  BSP default SOFT mode.  SOFT mode programs the BSP CCU coordination registers
  and starts the selected decoder core directly; HARD mode is opt-in and uses
  the linked-table CCU path below.
* Rock 5B decoder masters use one IOMMU-core-owned default DMA domain.  The
  secondary ``vdec1_mmu`` provider names ``vdec0_mmu`` with
  ``rockchip,shared-domain-owner``; the Rockchip IOMMU provider consequently
  returns the owner's singleton group for both decoder masters.  The generic
  IOMMU core allocates the normal DMA domain, attaches both hardware IOMMUs to
  it, and installs ordinary DMA ops on both masters.  Nodes without the opt-in
  property retain the existing one-group-per-provider behavior.  Self-links,
  chained owners, and unresolved/unregistered owners fail or defer provider
  probe rather than silently creating an unsafe topology.
* HARD-CCU DMA visibility is fail-closed.  Because the coordinator may dispatch
  any linked table to any online decoder core, all such cores must use the same
  DMA/IOMMU domain (or all operate without an IOMMU).  The rewrite compares the
  public ``iommu_domain`` identity recorded for every online core behind the
  coordinator before advertising that HARD-CCU decoder and again while building
  its all-core work mask.  A mixed-domain cluster is not advertised, descriptor
  preparation returns ``-EXDEV``, and peer power-up is limited to the validated
  descriptor mask/domain.  This prevents a peer IOMMU from fetching a coherent
  link-table IOVA or imported-buffer IOVA that exists only in the selected
  core's domain.  The Rock 5B shared-provider topology satisfies this invariant;
  the runtime identity check remains the fail-closed verifier for other or
  malformed device trees.  Missing link MMIO in requested HARD mode also fails
  probe instead of leaving an advertised but unusable decoder.
* RK3588 VDPU381/RKVDEC2 hard-CCU link-table capability data, backing
  allocation, table materialization, and readback copying.  Decoder cores with
  a ``link`` MMIO window and ``rockchip,ccu`` phandle validate the BSP VDPU381
  link IRQ register and allocate DMA-coherent per-task link-table nodes
  sized from ``rockchip,task-capacity`` using public DMA APIs.  Accepted
  decoder jobs reserve one node from a per-core bitmap, stage the BSP
  write/readback register partitions into that job-owned table node, hold a
  counted reference to the referenced CCU coordinator, and release both on
  completion or abort.  Both the write-source and readback-destination register
  spans are validated during table materialization, so an image truncated
  before the final BSP readback word fails with ``-EINVAL`` before the CCU
  start doorbell.  A HARD-mode pool must contain at least two nodes, and one
  unused node per core is retained as the hardware next-table sentinel instead
  of consuming the full pool and emitting a zero tail address.  The per-core
  active table list is relinked whenever a node is staged or released, so the
  tail points at that unused node and earlier active tables point at the next
  active table, matching the BSP add-mode queue invariant.  Jobs also record
  BSP-shaped hard-CCU submit descriptor values for
  all online cores behind the selected coordinator: core-work mask, table
  address, link-mode word,
  and autogate/work/cfg-done bits.  When the CCU is
  idle, the rewrite takes a separate chain-owned power reference on every
  online work-mask decoder core, including the selected core in addition to
  its per-job reference, programs each powered core's link window for CCU-work
  mode, mirrors the BSP
  fixed-RCB setup by writing the device-tree ``rockchip,rcb-info`` register
  layout once per core and setting the fixed-RCB link latch, starts that job
  through the hard-CCU config path, tracks the started job on the coordinator's
  running list, relinks that list as a coordinator-wide table chain, scans it on
  completion, and copies BSP table readback partitions back into the normal
  register image with the VDPU381 core interrupt-status word from the table
  interrupt-status word.  Hard-CCU register start/stop writes are serialized on
  the coordinator, and completion releases stop CCU work only after the
  coordinator running list drains.  If the job that powered the work-mask
  decoder cores completes while the coordinator still owns queued hard-CCU
  work, every chain-owned core reference, including that job's selected core,
  moves to the next listed CCU job and is released only when the chain drains
  or aborts.  If the CCU is already active and the rewrite has a
  tracked coordinator chain, additional jobs are appended with the BSP
  ``ADD_MODE`` link-mode bit.  HARD task submission writes userspace task
  registers only into the coherent descriptor table; it does not also write
  those spans or per-job cache controls into the software-selected core, which
  may be physically executing a peer-owned descriptor.  Per-core link/RCB
  setup is performed only while starting an idle coordinator, not for an
  add-mode job.  That idle start also applies the BSP cache-size, cache-clear,
  and max-outstanding-read setup to every powered work-mask core, since the
  coordinator may dispatch the first descriptor to any of them.  All required
  cache offsets are validated before the first write; a truncated core window
  fails the start instead of leaving a partially configured cache.  Missing
  descriptors or an untracked busy CCU are submission errors instead of
  starting the core outside HARD-CCU coordination.
  Because HARD-CCU execution can finish a table on a physical core other than
  the table's software-selected owner, the IRQ thread orders coherent DMA
  readback and scans the coordinator running list instead of completing the
  interrupting core's active slot.  It completes a table only after claiming
  that exact job from its owning software slot.  The threaded IRQ path takes a
  counted reference to that owner core and waits for its serialized submit or
  abort section before rechecking the slot, so an IRQ that races the start
  doorbell cannot be acknowledged and then abandoned until the 500 ms timeout,
  and concurrent completion/removal cannot free the owner during that handoff.
  If a completed table reports a VDPU381 error bit, the rewrite preserves the
  readback status for userspace, force-stops the coordinator, resets the
  reporting job's core, and aborts remaining active dependents to contain the
  failed chain.  The more selective
  relink/reset/resend path remains the timeout recovery policy described above.
* RK3588 VEPU580 hardware-watchdog programming before the encoder start
  doorbell.  After task registers and the configured core rate are applied,
  the rewrite derives the BSP 50/100/200/400/800 ms frame threshold from the
  programmed 8-pixel-unit width/height fields, preserves the submitted
  submodule-watchdog byte, and bounds the 24-bit frame threshold.  Resolutions
  above the BSP table use its longest interval rather than programming a zero
  threshold, while the independent 500 ms software timeout remains active.
* ``MPP_CMD_POLL_HW_IRQ`` for RK3588 RKVENC2 encoder slice result streaming.
  The rewrite advertises the forward-port ``POLL_BUTT`` command boundary,
  detects slice mode from the submitted RKVENC2 register image
  (slice-length FIFO plus slice split enabled), queues slice-length words from
  slice/done IRQs in a job-owned FIFO, copies up to userspace ``count_max``
  entries into the flexible poll buffer, and falls through to normal
  completion/readback handling on the final slice.  Non-split jobs polled
  through ``POLL_HW_IRQ`` use the same full-frame completion path as
  ``POLL_HW_FINISH`` without interpreting the slice-only flexible buffer;
  an empty session likewise returns ``-EIO`` before touching that buffer.
  Slice-FIFO overflow is reported with ``-EOVERFLOW`` once per observed
  overflow, then the latch clears so a retry can drain retained entries and
  eventually consume the completed job instead of permanently poisoning the
  session head.  A VEPU580 bitstream-overflow IRQ advances the circular
  bitstream write pointer by the BSP-defined 128-byte step (wrapping to the
  programmed bottom address), retains the overflow bit for final readback, and
  lets encoding continue.  When that frame reaches its terminal IRQ, the
  retained overflow participates in VEPU580's BSP ``0x03f0`` reset mask, so the
  core is reset before the next frame rather than continuing from
  overflow-recovery state.  For the BSP erratum combination of H.264 slice
  mode, a nonzero translated external line-buffer address, and the slice-flush
  bit, submit clears only the flush bit in the job-private register image before
  MMIO programming.  H.265 and jobs without an external line buffer retain the
  userspace value.
* ``MPP_CMD_SET_RCB_INFO`` for RK3588 RKVENC2/RKVDEC2 jobs.  The rewrite stores
  BSP-compatible ``(register index, size)`` descriptors per session, snapshots
  them into each job, allocates per-core coherent scratch memory using the
  device-tree ``rockchip,rcb-iova`` size, and patches matching registers after
  fd-to-IOVA translation and offset handling.  Out-of-range register indices
  are ignored without consuming scratch space.  Decoder RCB obeys the BSP
  ``rockchip,rcb-min-width`` gate using retained ``SEND_CODEC_INFO`` width.
  This uses the public DMA API rather than BSP fixed-IOVA SRAM reservation;
  DMA address zero remains a valid coherent allocation and is not mistaken for
  an absent scratch buffer, while an allocation whose full span exceeds the
  32-bit codec aperture fails probe with ``-EOVERFLOW``.
* Refcounted batch/session/hardware job ownership so a concurrent poll, reset,
  close, or IRQ thread does not free a job while another owner is still using
  it.  Queue admission is serialized with core/CCU removal, ensuring removal
  either rejects a prepared job or observes it in the queued-job abort sweep.
* Optional ``ROCKCHIP_MPP_REWRITE_KUNIT_TEST`` coverage for rewrite-local ABI
  parser helpers, including command range and per-message flag classification,
  command group boundary queries, fixed-width V1/``mpp_bat_msg`` ABI layout,
  V1-to-native request conversion, payload-copy classification,
  ``SEND_CODEC_INFO`` storage and trailing-byte tolerance, overflow-safe
  RKVDEC2 CCU timeout selection, VEPU580 resolution/rate watchdog selection,
  required-versus-virtual clock-count validation,
  per-match minimum MMIO sizing including the VDPU381 cache aperture, exact
  VEPU58x/VDPU38x hardware-ID gating,
  required CCU/core-mask topology,
  hot-reprobe core-ID vacancy and decoder-mask collision handling,
  register-span overflow
  checks, BSP VDPU381 link IRQ decoding/acknowledgment, link-table
  layout/materialization/readback-boundary/ownership/relinking, CCU-reference
  lifetime,
  hard-CCU running-list table-chain relinking/scanning and cross-core done-table
  detection, peer-core power ownership transfer, release-time peer-core
  ownership transfer, unfinished-chain relinking for hard-CCU resend
  preparation, unfinished-job collection for hard-CCU resend, active-slot retry
  preservation and IOMMU refresh accounting, hard-CCU table-status readback,
  hard-CCU idle/add-mode descriptor values, hard-CCU all-core work-mask
  selection and shared-DMA-domain gating, descriptor ownership publication
  before the start doorbell, fixed-RCB link-latch programming, fail-closed
  VDPU381 cache/max-read programming,
  decoder RCB min-width gating,
  RKVDEC2 ccu-mode normalization and SOFT-CCU register programming, CCU
  coordinator removal cleanup for queued and active dependent-core jobs,
  recovery-failed core/CCU admission and scheduling rejection,
  deferred peer-abort target replacement/result/reference lifetime and
  replacement-watchdog preservation, MPP
  core-counter and per-core timing routing, exact IOMMU fault source matching
  plus HARD-CCU descriptor-owner/fail-closed fallback selection, exact
  IOMMU-fault activation attribution, and replacement-watchdog preservation,
  exact timeout-job targeting and stale-worker replacement rearming,
  ``INIT_TRANS_TABLE`` ``u16`` storage, locking, and boundary behavior,
  mandatory built-in address-table union plus explicit-IOVA range/lifetime
  validation,
  immutable per-job session-state snapshots, idempotent client initialization
  plus rebind rejection, reset-time staged-job cancellation, stale-epoch
  admission rejection, atomic active-list/scheduler-queue publication, and
  reset/completion-safe job hardware pin/detach lifetime,
  ``RELEASE_FD`` import-cache sweeping across all DMA-device mappings for one
  fd while preserving other fd imports,
  ``SET_REG_ADDR_OFFSET`` tuple staging/apply behavior, cap handling, literal
  register wrap rejection, cumulative dma-buf/32-bit IOVA bounds, and
  coherent-allocation span bounds,
  procfs support-command table coverage,
  RKVENC2 DCHS tx/rx id remapping and release, independent-core DCHS id
  capacity plus occupied/exhausted admission errors, VEPU580 bitstream-overflow
  pointer advance/wrap and terminal reset classification, the H.264 external
  line-buffer slice-flush fixup, ``POLL_HW_IRQ``
  flexible-buffer sizing, RKVENC2 slice-mode detection and slice FIFO
  recoverable overflow/final-slice reporting, non-slice flexible-buffer bypass,
  ``POLL_HW_FINISH`` nonblocking pending-job
  ``-EAGAIN`` without consuming the active job, and ``SET_SESSION_FD``
  invalid-fd status, done-marker handling, batch job splitting,
  batch-server wait-layout recognition plus collector-level ``-EOPNOTSUPP``
  rejection without status-slot writeback, total-message cap enforcement, and
  public ``RESET_SESSION``/
  file-close cleanup of imports plus queued/active jobs.

Debugging
---------

The driver exposes a self-contained snapshot and a bounded recent-event journal
under ``/sys/kernel/debug/rk_mpp_rewrite``.  They do not require rebuilding with
extra logging and do not emit per-job kernel messages unless tracing is enabled.

``state``
  Reports aggregate job/error/IRQ counters, every bound encoder/decoder/CCU
  device (online/recovery-failed/runtime-PM/queue/active-job state and active
  age), the software dispatch queue with each job's queued age, and active
  RKVENC2 DCHS slots.  Counts include jobs discarded by reset/close, hardware
  reset attempts, and first reset failures that caused permanent quarantine.
  The ``io imports`` value, also exposed as ``import_count``, is a live mapping
  gauge rather than a cumulative counter and returns to zero after cached and
  job-held imports are released.
  Session and job ids are shared with the event journal so one userspace
  failure can be followed through the driver.

``events``
  Keeps the latest 64 request/poll rejection, queue, dispatch, hardware start,
  IRQ, completion, timeout, IOMMU-fault, abort, and spurious-IRQ events.  Each
  row includes a monotonic sequence, timestamp, device/core,
  session/job/client, errno, IRQ status, and event-specific data.  Writing ``1``
  clears the retained rows without resetting the monotonic sequence.

``trace_mask``
  Opt-in live kernel-log tracing of the same structured events: bit 0 is job
  lifecycle, bit 1 is IRQ, and bit 2 is rejection/error/recovery.  The default
  is zero.  Use ``7`` only for a short reproduction because successful
  high-throughput jobs intentionally generate several lifecycle records.

For a minimal failure capture::

  mount -t debugfs none /sys/kernel/debug  # if not already mounted
  echo 1 > /sys/kernel/debug/rk_mpp_rewrite/events
  # reproduce once
  cat /sys/kernel/debug/rk_mpp_rewrite/state
  cat /sys/kernel/debug/rk_mpp_rewrite/events

The event ``data`` column is currently the rejected command/request count or
register-word count for setup failures, the poll command for ``poll-fail``, the
queue depth for ``queued``, hardware elapsed nanoseconds for ``done``, a
hard-CCU completion-race flag for ``timeout``, and the faulting IOVA for
``iommu-fault``.

Recognized But Unsupported
--------------------------

* Dormant libmpp batch-server wait arrays, recognized as repeated
  ``SET_SESSION_FD`` + ``POLL_HW_FINISH|POLL_NON_BLOCK|LAST_MSG`` pairs, return
  ``-EOPNOTSUPP``.  Current libmpp does not wire callers to this server path,
  and supporting it would require a multi-``LAST_MSG`` ioctl behavior that the
  BSP collector does not expose for normal submissions.
* ``MPP_FLAGS_SECURE_MODE`` returns ``-EOPNOTSUPP`` before request processing.
  The fixed RK3588 rewrite has no secure dma-buf attachment, protected IOMMU
  domain, or secure-monitor submission path, so accepting the bit while running
  the job as ordinary DMA would be a fail-open ABI claim.  The checked current
  libmpp tree defines but does not send this flag.

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

Findings
--------

* **RKVDEC2 multi-core CCU mode mismatch is resolved at the code level; board
  validation is still required.**  The vendor RKVDEC2 driver supports two
  multi-core coordination modes chosen by the ``rockchip,ccu-mode`` device-tree
  property: SOFT CCU (``= 1``, software-scheduled; the driver writes each task's
  registers directly to the core MMIO and reaps completion in a software IRQ
  handler, with no in-memory descriptor list) and HARD CCU (``= 2``, a hardware
  linked-list of DMA config tables that the CCU walks autonomously across cores).
  The in-memory link table is a HARD-CCU-only construct.

  The shipped RK3588 device tree requests SOFT CCU
  (``rk3588-base.dtsi`` ``rkvdec_ccu`` node: ``/* 1: soft ccu  2: hw ccu */``
  ``rockchip,ccu-mode = <1>``; ``rk3588-rock-5b.dtsi`` enables that node without
  overriding the mode), matching the vendor driver's own SOFT default.  The
  rewrite now reads that property from the CCU/coordinator device and normalizes
  missing or invalid values to SOFT, so the Rock 5B DT no longer forces the
  rewrite down the HARD-CCU linked-list path.

  In SOFT mode, the rewrite prepares the same CCU task register words, programs
  the software CCU coordination registers, and starts the selected core directly.
  In HARD mode, it uses the RK3588 VDPU381/RKVDEC2 link-table allocation,
  materialization, coordinator running-list, add-mode, readback, timeout, and
  recovery machinery as an explicit opt-in path.  HARD mode additionally
  requires one DMA/IOMMU domain shared by every online decoder core behind the
  coordinator.  Independent per-core IOMMU providers do not establish that
  invariant by themselves, so selecting HARD mode without shared-domain
  integration disables decoder support instead of exposing peer cores to
  unmapped table/import IOVAs.  This tree's Rock 5B description supplies that
  integration explicitly: ``vdec1_mmu`` names ``vdec0_mmu`` through
  ``rockchip,shared-domain-owner``.  The shipped board still selects SOFT mode,
  so the HARD path remains opt-in and hardware-unvalidated.  KUnit covers
  ccu-mode normalization, SOFT-CCU register programming, the HARD-CCU
  shared-domain gate, and descriptor and ownership helpers, but these are
  logic-level tests: they do not drive MMIO, DMA, the real CCU register block,
  or real decoder interrupts.

  Remaining evidence required for release is RK3588 hardware validation of the
  default SOFT path under normal decode, multi-stream scheduling, timeout/reset,
  runtime suspend, and IOMMU-fault recovery, plus opt-in HARD-CCU validation if
  that mode is ever enabled by a board DT.
