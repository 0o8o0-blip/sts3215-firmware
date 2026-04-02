"""
Peripheral register stubs for Unicorn ARM emulation.

Provides smarter peripheral models than returning 0 for all reads.
Each peripheral auto-manages status bits (e.g. FMC BSY clears after
write operations, I2C state machine advances, UART TC/TBE auto-set).

Usage:
    stubs = PeripheralStubs()
    stubs.install(uc)   # hooks read/write on all peripheral regions

Then peripheral accesses during emulation will be handled correctly.
"""

import math
import struct

# ================================================================
# Peripheral base addresses (GD32F1x0)
# ================================================================
FMC_BASE      = 0x40022000
I2C0_BASE     = 0x40005400
USART1_BASE   = 0x40004400
DMA_BASE      = 0x40020000
TIMER0_BASE   = 0x40012C00
TIMER15_BASE  = 0x40014000
RCU_BASE      = 0x40021000
SYSCFG_BASE   = 0x40010000
FWDGT_BASE    = 0x40003000
ADC_BASE      = 0x40012400

GPIOA_BASE    = 0x48000000
GPIOB_BASE    = 0x48000400
GPIOF_BASE    = 0x48001400

SCB_BASE      = 0xE000E000
NVIC_BASE     = 0xE000E100


# ================================================================
# I2C AS5600 magnetic encoder peripheral model
# ================================================================
class I2CModel:
    """Models the I2C0 peripheral registers for AS5600 encoder simulation.

    Tracks the firmware's I2C transaction state machine as it writes to
    CTL0 (START/STOP/ACK bits) and DATA, and provides the correct STAT0,
    STAT1, and DATA register values that the i2c_event_handler expects.

    The AS5600 transaction sequence the firmware drives:
      1. START → STAT0.SB
      2. Write slave addr (write mode) → STAT0.ADDR
      3. Write register addr (0x0C) → STAT0.BTC
      4. Repeated START → STAT0.SB
      5. Write slave addr (read mode) → STAT0.ADDR
      6. ACK disable → STAT0.BTC with 2 data bytes available
    """

    # CTL0 bits
    _START = 0x100
    _STOP  = 0x200
    _ACK   = 0x400
    _POS   = 0x800

    # Transaction phases (what the peripheral is doing)
    IDLE          = 'idle'
    START_SENT    = 'start_sent'
    ADDR_W_ACKED  = 'addr_w_acked'
    REG_SENT      = 'reg_sent'
    RESTART_SENT  = 'restart_sent'
    ADDR_R_ACKED  = 'addr_r_acked'
    ACK_DISABLED  = 'ack_disabled'
    DATA_READY    = 'data_ready'
    COMPLETE      = 'complete'

    def __init__(self):
        self.encoder_angle = 2048   # Current simulated AS5600 value (12-bit)
        self.phase = self.IDLE
        self.slave_addr = 0
        self.reg_addr = 0
        self.ctl0 = 0               # Last CTL0 value
        self.ctl1 = 0               # Last CTL1 value
        self._addr_read_pending = False  # ADDR bit needs clearing via STAT1 read
        self._data_byte_idx = 0     # Which data byte to return next (0=high, 1=low)
        self._data_available = False # Data bytes ready to read from receive buffer

    def reset(self):
        """Reset to idle state."""
        self.phase = self.IDLE
        self.slave_addr = 0
        self.reg_addr = 0
        self.ctl0 = 0
        self.ctl1 = 0
        self._addr_read_pending = False
        self._data_byte_idx = 0
        self._data_available = False

    def handle_write(self, offset, value):
        """Called when firmware writes to I2C0 register at given offset."""
        if offset == 0x00:  # CTL0
            if value & self._STOP:
                self.phase = self.IDLE
                self._data_byte_idx = 0
                # STOP bit is self-clearing in hardware
                value &= ~self._STOP
            if value & self._START:
                if self.phase == self.REG_SENT:
                    # Repeated START after register address sent
                    self.phase = self.RESTART_SENT
                else:
                    self.phase = self.START_SENT
                # START bit is self-clearing in hardware
                value &= ~self._START
            if self.phase == self.ADDR_R_ACKED and not (value & self._ACK):
                # ACK disabled in read mode — peripheral will NACK after next byte
                self.phase = self.ACK_DISABLED
            self.ctl0 = value

        elif offset == 0x04:  # CTL1
            self.ctl1 = value

        elif offset == 0x10:  # DATA
            data_byte = value & 0xFF
            if self.phase == self.START_SENT:
                # Firmware sends slave address in write mode
                self.slave_addr = data_byte >> 1
                self.phase = self.ADDR_W_ACKED
                self._addr_read_pending = True
            elif self.phase == self.ADDR_W_ACKED:
                # Firmware sends register address
                self.reg_addr = data_byte
                self.phase = self.REG_SENT
            elif self.phase == self.RESTART_SENT:
                # Firmware sends slave address in read mode
                self.slave_addr = data_byte >> 1
                self.phase = self.ADDR_R_ACKED
                self._addr_read_pending = True
                self._data_byte_idx = 0

    def handle_read(self, offset):
        """Called when firmware reads I2C0 register. Returns register value."""
        if offset == 0x14:  # STAT0
            return self._get_stat0()
        elif offset == 0x18:  # STAT1
            return self._get_stat1()
        elif offset == 0x10:  # DATA
            return self._get_data()
        elif offset == 0x00:  # CTL0
            return self.ctl0
        elif offset == 0x04:  # CTL1
            return self.ctl1
        return 0

    def _get_stat0(self):
        """Return STAT0 register value based on current transaction phase."""
        if self.phase == self.IDLE:
            return 0
        elif self.phase == self.START_SENT:
            # SB (bit 0): start condition generated
            return 0x01
        elif self.phase == self.ADDR_W_ACKED:
            # ADDR (bit 1): address sent and ACKed
            # TBE (bit 7): transmit buffer empty
            self._addr_read_pending = True
            return 0x82
        elif self.phase == self.REG_SENT:
            # BTC (bit 2): byte transfer complete
            # TBE (bit 7): transmit buffer empty
            return 0x84
        elif self.phase == self.RESTART_SENT:
            # SB (bit 0): repeated start generated
            return 0x01
        elif self.phase == self.ADDR_R_ACKED:
            # ADDR (bit 1): address sent and ACKed in read mode
            self._addr_read_pending = True
            return 0x02
        elif self.phase == self.ACK_DISABLED:
            # After ACK disabled, data transfer completes
            # BTC (bit 2): both bytes received
            self.phase = self.DATA_READY
            self._data_available = True
            return 0x04
        elif self.phase == self.DATA_READY:
            # BTC (bit 2): transfer complete, data available
            return 0x44
        elif self.phase == self.COMPLETE:
            return 0
        return 0

    def _get_stat1(self):
        """Return STAT1 register value. Reading STAT1 after STAT0 clears ADDR."""
        val = 0x03  # MASTER (bit 0) + BUSY (bit 1)
        if self._addr_read_pending:
            self._addr_read_pending = False
            # ADDR flag is cleared by the STAT0+STAT1 read sequence
        if self.phase == self.IDLE:
            val = 0
        return val

    def _get_data(self):
        """Return DATA register value (encoder bytes in read mode).

        The firmware sends STOP before reading the data bytes from the
        receive buffer, so we must keep data available even after STOP/IDLE.
        We track _data_available to handle this.
        """
        if self._data_available:
            angle = self.encoder_angle & 0xFFF
            if self._data_byte_idx == 0:
                self._data_byte_idx = 1
                return (angle >> 8) & 0xFF  # High byte
            else:
                self._data_byte_idx = 0
                self._data_available = False
                return angle & 0xFF  # Low byte
        return 0


# ================================================================
# DC Motor physics model for closed-loop servo simulation
# ================================================================
class MotorModel:
    """STS3215 servo motor model with realistic electromechanical dynamics.

    Simulates a brushed DC motor through a 254:1 gearbox driving an AS5600
    magnetic encoder.  Models electrical dynamics (inductance, back-EMF,
    current limiting), mechanical dynamics (inertia, Coulomb/viscous friction,
    backlash dead zone, cogging torque), supply voltage sag under load,
    thermal RC network, and AS5600 encoder characteristics (quantization,
    hysteresis, propagation delay).

    Designed for deterministic closed-loop servo testing — uses a seeded RNG
    so both the original and reimplemented firmware see identical physics.

    The STS3215 H-bridge drive scheme:
      ch0cv=period, ch2cv=period  -> brake (both sides ON, motor shorted)
      ch0cv=0,      ch2cv=duty    -> forward rotation
      ch0cv=duty,   ch2cv=0       -> reverse rotation
      ch0cv=0,      ch2cv=0       -> coast (no drive)
    """

    def __init__(self, initial_position=2048, position=None):
        if position is not None:
            initial_position = position

        # --- Output shaft state (what the encoder sees) ---
        self.position = float(initial_position)  # encoder counts (0-4095)
        self.velocity = 0.0          # counts per tick

        # --- Motor-side state (behind the gearbox) ---
        self.motor_velocity = 0.0    # motor-side velocity (output-equivalent counts/tick)
        self.motor_position = float(initial_position)  # motor-side position for backlash
        self.motor_angle = 0.0       # accumulated motor angle (radians) for cogging

        # --- Motor electrical parameters (STS3215) ---
        # Calibrated to match STS3215 specs:
        #   No-load speed: ~60 RPM at output ≈ 4.1 counts/tick
        #   Stall torque: ~17 kg*cm = 1.67 Nm at output
        # ke and kt are in MODEL UNITS (not SI): motor_velocity is in
        # counts/tick * gear_ratio, NOT rad/s. This avoids unit conversion
        # in the hot loop.
        self.gear_ratio = 254.0      # motor:output gear ratio
        self.pwm_period = 1000       # TIMER0 CAR value
        self.nominal_voltage = 7.4   # battery nominal voltage (V)
        self.battery_resistance = 0.3  # battery + wiring internal resistance (ohms)
        self.kt = 0.00317            # torque constant (model units: Nm/A at output)
        self.ke = 0.00640            # back-EMF constant (model units: V per motor_velocity)
        self.resistance = 2.5        # motor winding resistance (ohms)
        self.max_current = 2.5       # H-bridge current limit (A)

        # --- Mechanical parameters ---
        self.motor_inertia = 2e-7    # motor rotor inertia (kg*m^2, coreless motor)
        self.load_inertia = 1e-4     # output shaft + load inertia (kg*m^2)
        self.viscous_friction = 0.002 # viscous friction coefficient (Nm*s)
        self.coulomb_friction = 0.02  # static/coulomb friction (Nm)
        self.stiction_threshold = 0.05  # velocity below which stiction applies (counts/tick)

        # --- Gearbox ---
        self.backlash = 0.5          # backlash dead zone in encoder counts
        self.gear_efficiency = 0.7   # gear train efficiency (0-1)

        # --- Cogging torque (brushed motor commutator ripple) ---
        self.cogging_amplitude = 0.001  # cogging torque at motor shaft (Nm)
        self.cogging_periods = 6     # torque ripple cycles per motor revolution

        # --- External forces ---
        self.external_torque = 0.0   # one-shot external load (Nm at output shaft)
        self.gravity_torque = 0.0    # constant gravity load (Nm)

        # --- AS5600 encoder model ---
        self.encoder_noise_std = 0.3  # encoder noise (counts, gaussian)
        self.encoder_glitch_prob = 0.001  # probability of random glitch per tick
        self.encoder_hysteresis = 1  # AS5600 hysteresis band (LSB)

        # --- Thermal model (RC network) ---
        self.motor_current = 0.0     # instantaneous motor current (A)
        self.temperature = 25.0      # motor winding temperature (deg C)
        self.ambient_temp = 25.0     # ambient temperature (deg C)
        self.thermal_resistance = 20.0  # winding-to-ambient (deg C/W)
        self.thermal_capacitance = 5.0  # winding thermal mass (J/deg C)

        # --- Internal state ---
        self._effective_voltage = self.nominal_voltage
        self._last_encoder = int(initial_position) & 0xFFF
        self._prev_position = float(initial_position)  # 1-tick delay buffer
        self._rng = __import__('random').Random(42)

    @property
    def supply_voltage(self):
        """Dynamic supply voltage after battery sag (back-compat property)."""
        return self._effective_voltage

    @supply_voltage.setter
    def supply_voltage(self, value):
        """Allow setting nominal voltage via legacy name."""
        self.nominal_voltage = value

    def step(self, ch0cv, ch2cv):
        """Advance motor state by one tick (1ms). Returns encoder reading (0-4095)."""
        dt = 0.001  # 1ms timestep

        # 1. Voltage sag: supply drops under load (from previous tick's current)
        self._effective_voltage = max(
            0.0, self.nominal_voltage - abs(self.motor_current) * self.battery_resistance)

        # 2. Compute applied voltage from PWM duty
        if ch0cv == self.pwm_period and ch2cv == self.pwm_period:
            applied_voltage = 0.0
            brake_mode = True
        elif ch0cv == 0 and ch2cv > 0:
            applied_voltage = (ch2cv / self.pwm_period) * self._effective_voltage
            brake_mode = False
        elif ch2cv == 0 and ch0cv > 0:
            applied_voltage = -(ch0cv / self.pwm_period) * self._effective_voltage
            brake_mode = False
        else:
            applied_voltage = 0.0
            brake_mode = False

        # 3. Motor electrical model (steady-state: I = (V - back_emf) / R)
        back_emf = self.ke * self.motor_velocity
        if brake_mode:
            motor_voltage = -back_emf
        else:
            motor_voltage = applied_voltage - back_emf

        self.motor_current = motor_voltage / self.resistance

        # 4. Current limiting (H-bridge hardware limit)
        self.motor_current = max(-self.max_current,
                                 min(self.max_current, self.motor_current))

        # 5. Motor torque + cogging
        motor_torque = self.kt * self.motor_current
        if self.cogging_amplitude > 0:
            cogging = self.cogging_amplitude * math.sin(
                self.cogging_periods * self.motor_angle)
            motor_torque += cogging

        # 6. Gear reduction: motor torque -> output torque
        output_torque = motor_torque * self.gear_ratio * self.gear_efficiency

        # 7. Friction model (Coulomb + viscous)
        driving_torque = output_torque + self.external_torque + self.gravity_torque
        if abs(self.velocity) < self.stiction_threshold:
            if abs(driving_torque) < self.coulomb_friction:
                friction_torque = -driving_torque
            else:
                friction_torque = -self.coulomb_friction * (1 if driving_torque > 0 else -1)
        else:
            sign = 1 if self.velocity > 0 else -1
            friction_torque = -(self.coulomb_friction * sign +
                                self.viscous_friction * self.velocity)

        # 8. Net torque and acceleration at output shaft
        net_torque = output_torque + friction_torque + self.external_torque + self.gravity_torque
        total_inertia = self.load_inertia + self.motor_inertia * self.gear_ratio ** 2
        acceleration = net_torque / total_inertia

        # 9. Semi-implicit Euler integration
        self.velocity += acceleration * dt
        self.motor_position += self.velocity
        self.motor_velocity = self.velocity * self.gear_ratio

        # 10. Motor angle tracking (for cogging torque)
        self.motor_angle += self.motor_velocity * dt * (2.0 * math.pi / 60.0)

        # 11. Backlash dead zone
        half_bl = self.backlash / 2.0
        if self.motor_position > self.position + half_bl:
            self.position = self.motor_position - half_bl
        elif self.motor_position < self.position - half_bl:
            self.position = self.motor_position + half_bl
        # else: motor is in dead zone, output doesn't move

        # 12. AS5600 encoder model: propagation delay + quantization + hysteresis
        delayed_pos = self._prev_position
        self._prev_position = self.position

        # Quantize to 12-bit
        quantized = int(round(delayed_pos)) % 4096

        # Hysteresis: only update if movement exceeds hysteresis band
        diff = (quantized - self._last_encoder + 2048) % 4096 - 2048  # signed wrap
        if abs(diff) > self.encoder_hysteresis:
            self._last_encoder = quantized

        encoder_pos = float(self._last_encoder)

        # Gaussian noise
        if self.encoder_noise_std > 0:
            encoder_pos += self._rng.gauss(0, self.encoder_noise_std)

        # Random glitch
        if self._rng.random() < self.encoder_glitch_prob:
            encoder_pos += self._rng.randint(-100, 100)

        # Wrap to 0-4095
        encoder_pos = encoder_pos % 4096

        # 13. Thermal model (RC network: winding temperature)
        power = self.motor_current ** 2 * self.resistance
        dT = (power - (self.temperature - self.ambient_temp) /
              self.thermal_resistance) / self.thermal_capacitance * dt
        self.temperature += dT

        # 14. Clear one-shot external torque
        self.external_torque = 0.0

        return int(encoder_pos) & 0xFFF

    def get_current_adc(self):
        """Return motor current as ADC value for current sense injection."""
        return int(abs(self.motor_current) * 0.5 / 3.3 * 4096) & 0xFFF

    def get_temperature_adc(self):
        """Return temperature as ADC value (NTC thermistor model)."""
        return int(max(0, min(4095, 3000 - (self.temperature - 25) * 30)))

    def get_voltage_adc(self):
        """Return supply voltage as ADC value (dynamic, includes sag).

        The STS3215 voltage divider scales ~7.4V to produce a reading of
        ~70 through adc_read_voltage's (raw<<4)>>8 formula. Calibrated
        so MIN_VOLTAGE=40 ≈ 4.2V (dead LiPo) and MAX_VOLTAGE=80 ≈ 8.5V.
        """
        return int(self._effective_voltage * 151.35) & 0xFFF


class PeripheralStubs:
    """Smart peripheral register models for GD32F1x0."""

    def __init__(self):
        # Register storage: address -> 32-bit value
        self.regs = {}
        # Write log: list of (address, size, value)
        self.write_log = []
        # I2C AS5600 encoder model
        self._i2c = I2CModel()
        # DMA transfer-complete tracking
        self._dma_transfers = {}  # ch_base -> (paddr, maddr, cnt)
        # UART TX buffer tracking
        self._uart_tx_buf = bytearray()
        # Instruction count for BSY auto-clear
        self._fmc_write_pending = False

    def clear(self):
        """Reset write log (keep register state)."""
        self.write_log = []

    def full_reset(self):
        """Reset everything."""
        self.regs = {}
        self.write_log = []
        self._i2c = I2CModel()
        self._dma_transfers = {}
        self._uart_tx_buf = bytearray()
        self._fmc_write_pending = False

    def set_i2c_encoder_data(self, value):
        """Set the encoder value that I2C reads will return."""
        self._i2c.encoder_angle = value & 0xFFF

    # ================================================================
    # Register read handler
    # ================================================================
    def read(self, address, size):
        """Return value for a peripheral register read."""
        # Align to 4-byte boundary for register lookup
        base = address & ~3
        val = self.regs.get(base, 0)

        # --- ADC CTL1: auto-clear calibration bits ---
        if base == ADC_BASE + 0x08:  # ADC_CTL1
            val &= ~0x0C  # Clear RSTCLB (bit3) and CLB (bit2)

        # --- FMC status: BSY always reads as 0 (operation complete) ---
        elif base == FMC_BASE + 0x0C:  # FMC_STAT
            val &= ~1  # Clear BSY bit

        # --- I2C0 registers: delegate to I2C model ---
        elif I2C0_BASE <= base < I2C0_BASE + 0x40:
            val = self._i2c.handle_read(base - I2C0_BASE)

        # --- USART1 STAT: always show ready ---
        elif base == USART1_BASE + 0x14:  # USART1 STAT
            # TBE=1 (bit 7), TC=1 (bit 6), IDLE=0 (bit 4), RBNE=0 (bit 5)
            val = 0xC0

        # --- USART1 RDATA ---
        elif base == USART1_BASE + 0x24:
            val = 0  # no data available

        # --- DMA INTF: transfer complete flags ---
        elif base == DMA_BASE + 0x00:  # DMA_INTF
            val = self.regs.get(base, 0)

        # --- RCU: return stored values (clock enables persist) ---
        elif RCU_BASE <= base < RCU_BASE + 0x100:
            val = self.regs.get(base, 0)
            # CTL: set PLL ready, HSI ready when enabled
            if base == RCU_BASE + 0x00:
                if val & 1:       # HSION -> HSIRDY
                    val |= 2
                if val & 0x01000000:  # PLLON -> PLLRDY
                    val |= 0x02000000
            # CFG0: reflect SWS from SW selection
            elif base == RCU_BASE + 0x04:
                sw = val & 0x03       # SW bits [1:0]
                val = (val & ~0x0C) | (sw << 2)  # SWS bits [3:2] = SW

        # --- FWDGT: status — return stored or default with flags set ---
        elif base == FWDGT_BASE + 0x0C:  # FWDGT_STAT
            val = self.regs.get(base, 0x07)  # PUD|RUD|WUD set by default

        # --- SCB/NVIC: return stored ---
        elif SCB_BASE <= base < SCB_BASE + 0x10000:
            val = self.regs.get(base, 0)

        # --- GPIO: return stored config ---
        elif 0x48000000 <= base < 0x48002000:
            val = self.regs.get(base, 0)

        # --- Default: return stored value ---
        else:
            val = self.regs.get(base, 0)

        # Extract the right bytes for sub-word reads
        if size == 1:
            shift = (address & 3) * 8
            return (val >> shift) & 0xFF
        elif size == 2:
            shift = (address & 2) * 8
            return (val >> shift) & 0xFFFF
        return val & 0xFFFFFFFF

    # ================================================================
    # Register write handler
    # ================================================================
    def write(self, address, size, value):
        """Handle a peripheral register write."""
        base = address & ~3
        self.write_log.append((address, size, value))

        # Merge sub-word writes into the 32-bit register
        if size == 1:
            shift = (address & 3) * 8
            mask = 0xFF << shift
            old = self.regs.get(base, 0)
            value = (old & ~mask) | ((value & 0xFF) << shift)
        elif size == 2:
            shift = (address & 2) * 8
            mask = 0xFFFF << shift
            old = self.regs.get(base, 0)
            value = (old & ~mask) | ((value & 0xFFFF) << shift)

        self.regs[base] = value & 0xFFFFFFFF

        # --- FMC CTL: auto-clear BSY after STRT ---
        if base == FMC_BASE + 0x10:  # FMC_CTL
            if value & 0x40:  # STRT bit
                # Clear BSY in STAT
                self.regs[FMC_BASE + 0x0C] = self.regs.get(FMC_BASE + 0x0C, 0) & ~1

        # --- FMC KEY: accept unlock sequences ---
        elif base == FMC_BASE + 0x04:  # FMC_KEY
            pass  # just store

        # --- I2C0 registers: delegate to I2C model ---
        elif I2C0_BASE <= base < I2C0_BASE + 0x40:
            self._i2c.handle_write(base - I2C0_BASE, value)

        # --- USART1 TDATA: capture transmitted bytes ---
        elif base == USART1_BASE + 0x28:
            self._uart_tx_buf.append(value & 0xFF)

        # --- USART1 ICR: clear flags ---
        elif base == USART1_BASE + 0x20:
            stat = self.regs.get(USART1_BASE + 0x14, 0xC0)
            stat &= ~value
            stat |= 0xC0  # TBE and TC always ready
            self.regs[USART1_BASE + 0x14] = stat

        # --- DMA channel registers: track enables ---
        elif DMA_BASE <= base < DMA_BASE + 0x100:
            # DMA INTC: clear interrupt flags
            if base == DMA_BASE + 0x04:
                intf = self.regs.get(DMA_BASE, 0)
                intf &= ~value
                self.regs[DMA_BASE] = intf

        # --- TIMER0 / TIMER15: persist values ---
        elif TIMER0_BASE <= base < TIMER0_BASE + 0x80:
            pass  # already stored

        elif TIMER15_BASE <= base < TIMER15_BASE + 0x80:
            # TIMER15 INTF: writing 0 to bit clears it
            if base == TIMER15_BASE + 0x10:
                pass  # stored as-is

        # --- GPIO: persist configuration ---
        elif 0x48000000 <= base < 0x48002000:
            pass  # already stored

        # --- RCU: persist ---
        elif RCU_BASE <= base < RCU_BASE + 0x100:
            pass  # already stored

        # --- SYSCFG ---
        elif SYSCFG_BASE <= base < SYSCFG_BASE + 0x100:
            pass  # stored

        # --- SCB / NVIC ---
        elif SCB_BASE <= base < SCB_BASE + 0x10000:
            # AIRCR write with SYSRESETREQ: trap
            if base == 0xE000ED0C and (value & 0x04):
                pass  # Don't actually reset, just record

    @property
    def i2c_model(self):
        """Access the I2C peripheral model directly."""
        return self._i2c

    # ================================================================
    # Unicorn hook installation
    # ================================================================
    def install(self, uc):
        """Install read/write hooks for all peripheral regions on a Unicorn engine."""
        from unicorn import UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE

        def _read_hook(uc, access, address, size, value, user_data):
            stubs = user_data
            val = stubs.read(address, size)
            if size == 1:
                uc.mem_write(address, struct.pack('<B', val & 0xFF))
            elif size == 2:
                uc.mem_write(address, struct.pack('<H', val & 0xFFFF))
            elif size == 4:
                uc.mem_write(address, struct.pack('<I', val & 0xFFFFFFFF))

        def _write_hook(uc, access, address, size, value, user_data):
            stubs = user_data
            stubs.write(address, size, value)

        # Peripheral region: 0x40000000 - 0x400FFFFF
        uc.hook_add(UC_HOOK_MEM_READ, _read_hook, self,
                    begin=0x40000000, end=0x400FFFFF)
        uc.hook_add(UC_HOOK_MEM_WRITE, _write_hook, self,
                    begin=0x40000000, end=0x400FFFFF)

        # GPIO region: 0x48000000 - 0x4800FFFF
        uc.hook_add(UC_HOOK_MEM_READ, _read_hook, self,
                    begin=0x48000000, end=0x4800FFFF)
        uc.hook_add(UC_HOOK_MEM_WRITE, _write_hook, self,
                    begin=0x48000000, end=0x4800FFFF)

        # SCB/NVIC region: 0xE000E000 - 0xE000EFFF
        uc.hook_add(UC_HOOK_MEM_READ, _read_hook, self,
                    begin=0xE000E000, end=0xE000EFFF)
        uc.hook_add(UC_HOOK_MEM_WRITE, _write_hook, self,
                    begin=0xE000E000, end=0xE000EFFF)

        # Store reference on UC for access from test code
        uc._periph_stubs = self

    # ================================================================
    # Convenience methods for test setup
    # ================================================================
    def seed_register(self, addr, value):
        """Pre-seed a register value."""
        self.regs[addr & ~3] = value & 0xFFFFFFFF

    def get_register(self, addr):
        """Read back a register value."""
        return self.regs.get(addr & ~3, 0)

    def get_write_log(self):
        """Return the write log as a list of (addr, size, value) tuples."""
        return list(self.write_log)

    def get_writes_to(self, base, size=0x100):
        """Return writes within a specific address range."""
        return [(a, s, v) for a, s, v in self.write_log
                if base <= a < base + size]
