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
* ``MPP_CMD_SET_SESSION_FD`` session switching, restricted to other
  ``/dev/mpp_service`` file descriptors.
* RK3588 BSP-style RKVENC2/RKVDEC2 platform-device binding with devm-managed
  MMIO, IRQ, clock, and reset discovery.
* ``MPP_CMD_QUERY_HW_SUPPORT`` from bound RK3588 MPP hardware cores.
* ``MPP_CMD_QUERY_HW_ID`` returns the register-0 hardware id captured from the
  bound RK3588 RKVENC2/RKVDEC2 core at probe time, matching the forward port's
  userspace-visible HAL-selection contract.
* ``MPP_CMD_QUERY_CMD_SUPPORT`` group boundary queries.
* Minimal ``/proc/mpp_service/supports-cmd`` and
  ``/proc/mpp_service/support_cmd`` discovery markers so current
  ``mpp-rockchip`` enables command probing.  These procfs files are read-only
  compatibility markers, not BSP debug/control ABI.
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
* Prepared register jobs select and hold a counted reference to an online
  RK3588 hardware core.  Kernel-translated jobs prefer an idle matching core;
  jobs flagged ``MPP_FLAGS_REG_FD_NO_TRANS`` stay on the default matching core
  so explicit ``TRANS_FD_TO_IOVA`` results remain device-consistent.  Core
  removal unlists the device and waits for prepared job references to drain
  before devm-managed resources are released.
* Pending-job poll/abort lifetime infrastructure: accepted jobs remain on the
  session pending list until ``POLL_HW_FINISH`` consumes the completed result
  or reset/close aborts the session.
* First RK3588 RKVENC2/RKVDEC2 hardware execution slice: one active job per
  bound core, runtime-PM power-domain resume, bulk clock enable, range-checked
  MMIO writes from the original ``SET_REG_WRITE`` spans, start-register
  deferral, IRQ-driven completion, retained ``SET_REG_READ`` register readback,
  BSP-style interrupt-status readback override, and decoder RLC decoded-length
  adjustment.
* Contended submits against the selected hardware core wait interruptibly for
  the core to become idle instead of returning ``-EBUSY``.
* Per-core software timeout completion for active RKVENC2/RKVDEC2 jobs using
  the same 500 ms timeout window as the BSP-derived forward port.  A timed-out
  job is removed from the active hardware slot, the core reset line is pulsed
  when available, runtime PM/clocks are released, ``POLL_HW_FINISH`` wakes, and
  the job returns ``-ETIMEDOUT``.
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
  fd-to-IOVA translation and offset handling.  Decoder RCB obeys the BSP
  ``rockchip,rcb-min-width`` gate using retained ``SEND_CODEC_INFO`` width.
  This uses the public DMA API rather than BSP fixed-IOVA SRAM reservation.
* Refcounted batch/session/hardware job ownership so a concurrent poll, reset,
  close, or IRQ thread does not free a job while another owner is still using it.
* Optional ``ROCKCHIP_MPP_REWRITE_KUNIT_TEST`` coverage for rewrite-local ABI
  parser helpers, including command range classification, command group
  boundary queries, payload-copy classification, register-span overflow checks,
  ``POLL_HW_IRQ`` flexible-buffer sizing, and RKVENC2 slice-mode detection.

Recognized But Unsupported
--------------------------

* No required RK3588 MPP userspace command is intentionally left in the
  recognized-but-unsupported bucket.

Outside This Slice
------------------

* Full BSP-equivalent RK3588 scheduling beyond simple idle-core selection:
  queued software scheduling separate from the submitter wait path, dual-core
  CCU policy, and SRAM-backed fixed-IOVA RCB optimization.
* Full BSP-equivalent timeout recovery policy and IOMMU fault recovery,
  including shared reset-domain serialization and MMU-domain refresh.
* Decoder performance-selector readback ranges above the core MMIO resource.
* Fence export/import semantics.
* 32-bit compat structure translation beyond the BSP-style shared ioctl path.
* ``MPP_IOC_CFG_V2``; the BSP-derived 6.18 driver also rejects this in the
  observed path.
