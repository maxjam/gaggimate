import PropTypes from 'prop-types';
import { Trans, useLingui } from '@lingui/react/macro';
import {
  PUMP_STATE,
  PUMP_VESC_IDS,
  erpmToRpm,
  formatAgo,
  formatFault,
  formatNumber,
  pumpStateInfo,
  pumpStopText,
} from '../../../utils/pumpDrive.js';

// Fork (maxjam.15): both pumps on the Flipsky dual VESC, live from the CAN bus (STATUS 1 / 4 / 5 and the
// fault poll), plus the last pump stop. One column per pump so two pumps in series can be compared line by line.
// The state is always written out; colour only repeats it.
const TONE_CLASS = {
  good: 'bg-success text-success-content',
  bad: 'bg-error text-error-content',
  warn: 'bg-warning text-warning-content',
  neutral: 'bg-base-content/10 text-base-content/70',
};

const missing = value => value === null || value === undefined;

function Row({ label, cells, main = false }) {
  return (
    <>
      <dt className='text-base-content/50 self-center text-[0.6rem] font-semibold tracking-wider whitespace-nowrap uppercase'>
        {label}
      </dt>
      {cells.map((cell, i) => (
        <dd
          key={i}
          title={cell.text}
          className={`min-w-0 truncate tabular-nums ${main ? 'text-base leading-tight font-semibold' : 'text-xs'} ${
            cell.alert ? 'text-error font-semibold' : main ? '' : 'text-base-content/70'
          }`}
        >
          {cell.text}
        </dd>
      ))}
    </>
  );
}

Row.propTypes = {
  label: PropTypes.string.isRequired,
  cells: PropTypes.arrayOf(PropTypes.shape({ text: PropTypes.string, alert: PropTypes.bool }))
    .isRequired,
  main: PropTypes.bool,
};

export function PumpDriveCard({ pumpDrive, inCard = false }) {
  const { t } = useLingui();
  if (!pumpDrive) return null;
  const limit = pumpDrive.lim ?? 0;
  // Pump 2 shows once it is switched on in the settings or its VESC is seen on the bus.
  const shown = (pumpDrive.p ?? [])
    .map((pump, index) => ({ pump, index }))
    .filter(({ pump, index }) => index === 0 || pump.s !== PUMP_STATE.NOT_USED || !missing(pump.e));
  const stop = pumpDrive.stop;
  const dutyText = `${formatNumber((pumpDrive.cd ?? 0) * 100, 1)} %`;
  const cells = fn => shown.map(({ pump }) => fn(pump));

  return (
    <div
      className={
        inCard ? 'flex flex-col gap-2' : 'card bg-base-100 flex flex-col gap-2 rounded-xl p-3'
      }
    >
      <div className='flex items-center justify-between'>
        <div className='text-base-content/50 text-[0.6rem] font-semibold tracking-wider uppercase'>
          <Trans>Pumps</Trans>
        </div>
        {pumpDrive.ok && (
          <div className='text-base-content/60 text-xs tabular-nums'>
            <Trans>Duty {dutyText}</Trans>
          </div>
        )}
      </div>

      {stop?.pending && (
        <div className='bg-error text-error-content rounded-lg p-2 text-xs' role='alert'>
          <span className='font-bold'>
            <Trans>Pumps stopped.</Trans>
          </span>{' '}
          {pumpStopText(stop)} <Trans>Wake the machine to carry on.</Trans>
        </div>
      )}
      {!pumpDrive.ok && (
        <div className='bg-warning text-warning-content rounded-lg p-2 text-xs' role='status'>
          <Trans>No pump readings from the controller.</Trans>
        </div>
      )}
      {pumpDrive.ok && pumpDrive.bus !== 0 && (
        <div className='bg-warning text-warning-content rounded-lg p-2 text-xs' role='status'>
          <Trans>CAN bus is down (bus-off). Check the wiring and the 500K bit rate.</Trans>
        </div>
      )}

      {pumpDrive.ok && shown.length > 0 && (
        <dl
          className='bg-base-200/60 grid items-baseline gap-x-3 gap-y-1 rounded-lg p-3'
          style={{ gridTemplateColumns: `auto repeat(${shown.length}, minmax(0, 1fr))` }}
        >
          <div aria-hidden='true' />
          {shown.map(({ pump, index }) => {
            const state = pumpStateInfo(pump.s);
            const number = index + 1;
            return (
              <div key={index} className='mb-1 flex min-w-0 flex-col items-start gap-1'>
                <div className='max-w-full truncate text-sm font-semibold'>
                  <Trans>Pump {number}</Trans>
                  <span className='text-base-content/50 ml-1 text-xs font-normal'>
                    ID {PUMP_VESC_IDS[index]}
                  </span>
                </div>
                <span
                  className={`rounded-full px-2 py-0.5 text-[0.65rem] leading-tight font-bold tracking-wide ${TONE_CLASS[state.tone]}`}
                >
                  {state.label}
                </span>
              </div>
            );
          })}
          <Row
            main
            label={t`RPM`}
            cells={cells(pump => ({ text: formatNumber(erpmToRpm(pump.e)) }))}
          />
          <Row main label={t`Motor A`} cells={cells(pump => ({ text: formatNumber(pump.a, 1) }))} />
          <Row
            main
            label={t`Motor °C`}
            cells={cells(pump => ({
              text: formatNumber(pump.mt, 1),
              alert: limit > 0 && !missing(pump.mt) && pump.mt >= limit,
            }))}
          />
          <Row label={t`FET °C`} cells={cells(pump => ({ text: formatNumber(pump.ft, 1) }))} />
          <Row
            label={t`Input`}
            cells={cells(pump => ({
              text: missing(pump.v)
                ? '—'
                : `${formatNumber(pump.v, 1)} V${missing(pump.ia) ? '' : ` · ${formatNumber(pump.ia, 1)} A`}`,
            }))}
          />
          <Row
            label={t`Fault`}
            cells={cells(pump => ({
              text: formatFault(pump.f),
              alert: !missing(pump.f) && pump.f !== 0,
            }))}
          />
        </dl>
      )}

      {stop && !stop.pending && (
        <div className='text-base-content/50 text-[0.65rem] leading-tight'>
          <Trans>Last stop:</Trans> {pumpStopText(stop)} ({formatAgo(stop.ago)})
        </div>
      )}
    </div>
  );
}

PumpDriveCard.propTypes = {
  pumpDrive: PropTypes.shape({
    ok: PropTypes.bool,
    bus: PropTypes.number,
    txf: PropTypes.number,
    cd: PropTypes.number,
    dual: PropTypes.bool,
    lim: PropTypes.number,
    trip: PropTypes.number,
    tp: PropTypes.number,
    p: PropTypes.arrayOf(
      PropTypes.shape({
        s: PropTypes.number,
        e: PropTypes.number,
        a: PropTypes.number,
        d: PropTypes.number,
        mt: PropTypes.number,
        ft: PropTypes.number,
        ia: PropTypes.number,
        v: PropTypes.number,
        f: PropTypes.number,
      }),
    ),
    stop: PropTypes.shape({
      c: PropTypes.number,
      p: PropTypes.number,
      d: PropTypes.number,
      ago: PropTypes.number,
      pending: PropTypes.bool,
    }),
  }),
  inCard: PropTypes.bool,
};
