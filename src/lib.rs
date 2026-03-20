#![allow(unexpected_cfgs)]

use embassy_time::Timer;
use esp_idf_matter::matter::import;

import!(DoorLock);
use std::{
    cell::{Cell, RefCell},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

use esp_idf_svc::hal::gpio::{InputOutput, Level, PinDriver};
use rs_matter::{
    dm::{Dataver, EndptId},
    tlv::Nullable,
};

use crate::door_lock::*;

#[derive(Debug, Copy, Clone)]
enum LockState {
    Locked,
    UnlockedUntil(Instant),
}

pub struct SolenoidHandler {
    dataver: Dataver,
    endpoint_id: EndptId,
    state: Cell<LockState>,
    relays: RefCell<(
        PinDriver<'static, InputOutput>,
        PinDriver<'static, InputOutput>,
    )>,
    unlock_request: Arc<AtomicBool>,
}

impl SolenoidHandler {
    pub fn new(
        dataver: Dataver,
        endpoint_id: EndptId,
        relay_a: PinDriver<'static, InputOutput>,
        relay_b: PinDriver<'static, InputOutput>,
        unlock_request: Arc<AtomicBool>,
    ) -> Self {
        Self {
            dataver,
            endpoint_id,
            state: Cell::new(LockState::Locked),
            relays: RefCell::new((relay_a, relay_b)),
            unlock_request,
        }
    }

    fn notify(&self, ctx: &impl rs_matter::dm::HandlerContext) {
        self.dataver_changed();
        ctx.notify_attribute_changed(
            self.endpoint_id,
            Self::CLUSTER.id,
            AttributeId::LockState as _,
        );
    }
}

const DEFAULT_FULL_UNLOCK_TIME: Duration = Duration::from_secs(10);
const FORCE_NOTIFY_INTERVAL: Duration = Duration::from_mins(1);

impl ClusterAsyncHandler for SolenoidHandler {
    #[doc = "The cluster-metadata corresponding to this handler trait."]
    const CLUSTER: rs_matter::dm::Cluster<'static> = FULL_CLUSTER
        .with_revision(7)
        .with_attrs(esp_idf_matter::matter::with!(
            AttributeId::LockState | AttributeId::LockType | AttributeId::AutoRelockTime
        ))
        .with_cmds(esp_idf_matter::matter::with!(
            CommandId::UnlockWithTimeout | CommandId::UnlockDoor | CommandId::LockDoor
        ));

    fn dataver(&self) -> u32 {
        self.dataver.get()
    }

    fn dataver_changed(&self) {
        self.dataver.changed();
    }

    async fn run(
        &self,
        ctx: impl rs_matter::dm::HandlerContext,
    ) -> Result<(), rs_matter::error::Error> {
        self.notify(&ctx);
        let mut last_force_notify = Instant::now();

        loop {
            // Check for external unlock requests (e.g. from WebSocket)
            if self.unlock_request.swap(false, Ordering::Relaxed) {
                log::info!("External unlock request received");
                self.state.set(LockState::UnlockedUntil(
                    Instant::now() + DEFAULT_FULL_UNLOCK_TIME,
                ));
            }

            let new_relay_state = match self.state.get() {
                LockState::Locked => Level::Low,
                LockState::UnlockedUntil(time) => {
                    if time < Instant::now() {
                        self.state.set(LockState::Locked);
                        Level::Low
                    } else {
                        Level::High
                    }
                }
            };

            if let Ok(ref mut itm) = self.relays.try_borrow_mut() {
                let is_changed = itm.0.get_level() != new_relay_state;
                if is_changed {
                    self.notify(&ctx);
                }

                itm.0.set_level(new_relay_state).expect("set relay A");
                itm.1.set_level(new_relay_state).expect("set relay B");
            }

            if last_force_notify.elapsed() >= FORCE_NOTIFY_INTERVAL {
                self.notify(&ctx);
                last_force_notify = Instant::now();
            }

            Timer::after_millis(100).await;
        }
    }

    async fn auto_relock_time(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<u32, rs_matter::error::Error> {
        Ok(DEFAULT_FULL_UNLOCK_TIME.as_secs() as u32)
    }

    async fn set_auto_relock_time(
        &self,
        _ctx: impl rs_matter::dm::WriteContext,
        _value: u32,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn lock_state(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<rs_matter::tlv::Nullable<DlLockState>, rs_matter::error::Error> {
        Ok(Nullable::some(match self.state.get() {
            LockState::Locked => DlLockState::Locked,
            LockState::UnlockedUntil(_) => DlLockState::Unlocked,
        }))
    }

    async fn lock_type(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<DlLockType, rs_matter::error::Error> {
        Ok(DlLockType::Magnetic)
    }

    async fn actuator_enabled(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<bool, rs_matter::error::Error> {
        Ok(true)
    }

    async fn operating_mode(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<OperatingModeEnum, rs_matter::error::Error> {
        Ok(OperatingModeEnum::Normal)
    }

    async fn supported_operating_modes(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<DlSupportedOperatingModes, rs_matter::error::Error> {
        Ok(DlSupportedOperatingModes::NORMAL)
    }

    async fn set_operating_mode(
        &self,
        _ctx: impl rs_matter::dm::WriteContext,
        _value: OperatingModeEnum,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_lock_door(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: LockDoorRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        self.state.set(LockState::Locked);
        Ok(())
    }

    async fn handle_unlock_door(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: UnlockDoorRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        self.state.set(LockState::UnlockedUntil(
            Instant::now() + DEFAULT_FULL_UNLOCK_TIME,
        ));
        Ok(())
    }

    async fn handle_unlock_with_timeout(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        request: UnlockWithTimeoutRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        let duration = request
            .timeout()
            .map(|sec| Duration::from_secs(sec as u64))
            .unwrap_or(DEFAULT_FULL_UNLOCK_TIME);
        self.state
            .set(LockState::UnlockedUntil(Instant::now() + duration));
        Ok(())
    }

    async fn handle_set_week_day_schedule(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: SetWeekDayScheduleRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_get_week_day_schedule<P: rs_matter::tlv::TLVBuilderParent>(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: GetWeekDayScheduleRequest<'_>,
        _response: GetWeekDayScheduleResponseBuilder<P>,
    ) -> Result<P, rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_clear_week_day_schedule(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: ClearWeekDayScheduleRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_set_year_day_schedule(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: SetYearDayScheduleRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_get_year_day_schedule<P: rs_matter::tlv::TLVBuilderParent>(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: GetYearDayScheduleRequest<'_>,
        _response: GetYearDayScheduleResponseBuilder<P>,
    ) -> Result<P, rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_clear_year_day_schedule(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: ClearYearDayScheduleRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_set_holiday_schedule(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: SetHolidayScheduleRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_get_holiday_schedule<P: rs_matter::tlv::TLVBuilderParent>(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: GetHolidayScheduleRequest<'_>,
        _response: GetHolidayScheduleResponseBuilder<P>,
    ) -> Result<P, rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_clear_holiday_schedule(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: ClearHolidayScheduleRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_set_user(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: SetUserRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_get_user<P: rs_matter::tlv::TLVBuilderParent>(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: GetUserRequest<'_>,
        _response: GetUserResponseBuilder<P>,
    ) -> Result<P, rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_clear_user(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: ClearUserRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_set_credential<P: rs_matter::tlv::TLVBuilderParent>(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: SetCredentialRequest<'_>,
        _response: SetCredentialResponseBuilder<P>,
    ) -> Result<P, rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_get_credential_status<P: rs_matter::tlv::TLVBuilderParent>(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: GetCredentialStatusRequest<'_>,
        _response: GetCredentialStatusResponseBuilder<P>,
    ) -> Result<P, rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_clear_credential(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: ClearCredentialRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_unbolt_door(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: UnboltDoorRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_set_aliro_reader_config(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: SetAliroReaderConfigRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }

    async fn handle_clear_aliro_reader_config(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
    ) -> Result<(), rs_matter::error::Error> {
        Err(rs_matter::error::Error::new(
            rs_matter::error::ErrorCode::InvalidAction,
        ))
    }
}
