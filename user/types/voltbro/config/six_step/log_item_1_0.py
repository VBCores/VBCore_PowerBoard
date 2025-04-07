# AUTOGENERATED, DO NOT EDIT.
#
# Source file:
# /home/pi/cyphal-types/voltbro/config/six_step/log_item.1.0.dsdl
#
# Generated at:  2024-03-29 15:42:04.214979 UTC
# Is deprecated: no
# Fixed port ID: None
# Full name:     voltbro.config.six_step.log_item
# Version:       1.0
#
# pylint: skip-file
# mypy: warn_unused_ignores=False

from __future__ import annotations
from nunavut_support import Serializer as _Serializer_, Deserializer as _Deserializer_, API_VERSION as _NSAPIV_
import numpy as _np_
from numpy.typing import NDArray as _NDArray_
import pydsdl as _pydsdl_
import uavcan.primitive.scalar
import voltbro.config.six_step

if _NSAPIV_[0] != 1:
    raise RuntimeError(
        f"Incompatible Nunavut support API version: support { _NSAPIV_ }, package (1, 0, 0)"
    )

def _restore_constant_(encoded_string: str) -> object:
    import pickle, gzip, base64
    return pickle.loads(gzip.decompress(base64.b85decode(encoded_string)))

# noinspection PyUnresolvedReferences, PyPep8, PyPep8Naming, SpellCheckingInspection, DuplicatedCode
class log_item_1_0:
    """
    Generated property settings use relaxed type signatures, accepting a large variety of
    possible representations of the value, which are automatically converted to a well-defined
    internal representation. When accessing a property, this strict well-defined internal
    representation is always returned. The implicit strictification enables more precise static
    type analysis.

    The value returned by the __repr__() method may be invariant to some of the field values,
    and its format is not guaranteed to be stable. Therefore, the returned string representation
    can be used only for displaying purposes; any kind of automation build on top of that will
    be fragile and prone to mismaintenance.
    """
    def __init__(self,
                 is_on:           None | uavcan.primitive.scalar.Integer8_1_0 = None,
                 is_stalling:     None | uavcan.primitive.scalar.Integer8_1_0 = None,
                 target_speed:    None | uavcan.primitive.scalar.Real32_1_0 = None,
                 speed:           None | uavcan.primitive.scalar.Real32_1_0 = None,
                 elec_theta:      None | uavcan.primitive.scalar.Real32_1_0 = None,
                 PWM:             None | uavcan.primitive.scalar.Integer32_1_0 = None,
                 I_A:             None | uavcan.primitive.scalar.Real32_1_0 = None,
                 I_B:             None | uavcan.primitive.scalar.Real32_1_0 = None,
                 I_C:             None | uavcan.primitive.scalar.Real32_1_0 = None,
                 velocity_report: None | voltbro.config.six_step.pid_report_1_0 = None,
                 current_report:  None | voltbro.config.six_step.pid_report_1_0 = None) -> None:
        """
        voltbro.config.six_step.log_item.1.0
        Raises ValueError if any of the primitive values are outside the permitted range, regardless of the cast mode.
        :param is_on:           uavcan.primitive.scalar.Integer8.1.0 is_on
        :param is_stalling:     uavcan.primitive.scalar.Integer8.1.0 is_stalling
        :param target_speed:    uavcan.primitive.scalar.Real32.1.0 target_speed
        :param speed:           uavcan.primitive.scalar.Real32.1.0 speed
        :param elec_theta:      uavcan.primitive.scalar.Real32.1.0 elec_theta
        :param PWM:             uavcan.primitive.scalar.Integer32.1.0 PWM
        :param I_A:             uavcan.primitive.scalar.Real32.1.0 I_A
        :param I_B:             uavcan.primitive.scalar.Real32.1.0 I_B
        :param I_C:             uavcan.primitive.scalar.Real32.1.0 I_C
        :param velocity_report: voltbro.config.six_step.pid_report.1.0 velocity_report
        :param current_report:  voltbro.config.six_step.pid_report.1.0 current_report
        """
        self._is_on:           uavcan.primitive.scalar.Integer8_1_0
        self._is_stalling:     uavcan.primitive.scalar.Integer8_1_0
        self._target_speed:    uavcan.primitive.scalar.Real32_1_0
        self._speed:           uavcan.primitive.scalar.Real32_1_0
        self._elec_theta:      uavcan.primitive.scalar.Real32_1_0
        self._PWM:             uavcan.primitive.scalar.Integer32_1_0
        self._I_A:             uavcan.primitive.scalar.Real32_1_0
        self._I_B:             uavcan.primitive.scalar.Real32_1_0
        self._I_C:             uavcan.primitive.scalar.Real32_1_0
        self._velocity_report: voltbro.config.six_step.pid_report_1_0
        self._current_report:  voltbro.config.six_step.pid_report_1_0

        if is_on is None:
            self.is_on = uavcan.primitive.scalar.Integer8_1_0()
        elif isinstance(is_on, uavcan.primitive.scalar.Integer8_1_0):
            self.is_on = is_on
        else:
            raise ValueError(f'is_on: expected uavcan.primitive.scalar.Integer8_1_0 '
                             f'got {type(is_on).__name__}')

        if is_stalling is None:
            self.is_stalling = uavcan.primitive.scalar.Integer8_1_0()
        elif isinstance(is_stalling, uavcan.primitive.scalar.Integer8_1_0):
            self.is_stalling = is_stalling
        else:
            raise ValueError(f'is_stalling: expected uavcan.primitive.scalar.Integer8_1_0 '
                             f'got {type(is_stalling).__name__}')

        if target_speed is None:
            self.target_speed = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(target_speed, uavcan.primitive.scalar.Real32_1_0):
            self.target_speed = target_speed
        else:
            raise ValueError(f'target_speed: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(target_speed).__name__}')

        if speed is None:
            self.speed = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(speed, uavcan.primitive.scalar.Real32_1_0):
            self.speed = speed
        else:
            raise ValueError(f'speed: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(speed).__name__}')

        if elec_theta is None:
            self.elec_theta = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(elec_theta, uavcan.primitive.scalar.Real32_1_0):
            self.elec_theta = elec_theta
        else:
            raise ValueError(f'elec_theta: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(elec_theta).__name__}')

        if PWM is None:
            self.PWM = uavcan.primitive.scalar.Integer32_1_0()
        elif isinstance(PWM, uavcan.primitive.scalar.Integer32_1_0):
            self.PWM = PWM
        else:
            raise ValueError(f'PWM: expected uavcan.primitive.scalar.Integer32_1_0 '
                             f'got {type(PWM).__name__}')

        if I_A is None:
            self.I_A = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(I_A, uavcan.primitive.scalar.Real32_1_0):
            self.I_A = I_A
        else:
            raise ValueError(f'I_A: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(I_A).__name__}')

        if I_B is None:
            self.I_B = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(I_B, uavcan.primitive.scalar.Real32_1_0):
            self.I_B = I_B
        else:
            raise ValueError(f'I_B: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(I_B).__name__}')

        if I_C is None:
            self.I_C = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(I_C, uavcan.primitive.scalar.Real32_1_0):
            self.I_C = I_C
        else:
            raise ValueError(f'I_C: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(I_C).__name__}')

        if velocity_report is None:
            self.velocity_report = voltbro.config.six_step.pid_report_1_0()
        elif isinstance(velocity_report, voltbro.config.six_step.pid_report_1_0):
            self.velocity_report = velocity_report
        else:
            raise ValueError(f'velocity_report: expected voltbro.config.six_step.pid_report_1_0 '
                             f'got {type(velocity_report).__name__}')

        if current_report is None:
            self.current_report = voltbro.config.six_step.pid_report_1_0()
        elif isinstance(current_report, voltbro.config.six_step.pid_report_1_0):
            self.current_report = current_report
        else:
            raise ValueError(f'current_report: expected voltbro.config.six_step.pid_report_1_0 '
                             f'got {type(current_report).__name__}')

    @property
    def is_on(self) -> uavcan.primitive.scalar.Integer8_1_0:
        """
        uavcan.primitive.scalar.Integer8.1.0 is_on
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._is_on

    @is_on.setter
    def is_on(self, x: uavcan.primitive.scalar.Integer8_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Integer8_1_0):
            self._is_on = x
        else:
            raise ValueError(f'is_on: expected uavcan.primitive.scalar.Integer8_1_0 got {type(x).__name__}')

    @property
    def is_stalling(self) -> uavcan.primitive.scalar.Integer8_1_0:
        """
        uavcan.primitive.scalar.Integer8.1.0 is_stalling
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._is_stalling

    @is_stalling.setter
    def is_stalling(self, x: uavcan.primitive.scalar.Integer8_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Integer8_1_0):
            self._is_stalling = x
        else:
            raise ValueError(f'is_stalling: expected uavcan.primitive.scalar.Integer8_1_0 got {type(x).__name__}')

    @property
    def target_speed(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 target_speed
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._target_speed

    @target_speed.setter
    def target_speed(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._target_speed = x
        else:
            raise ValueError(f'target_speed: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    @property
    def speed(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 speed
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._speed

    @speed.setter
    def speed(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._speed = x
        else:
            raise ValueError(f'speed: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    @property
    def elec_theta(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 elec_theta
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._elec_theta

    @elec_theta.setter
    def elec_theta(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._elec_theta = x
        else:
            raise ValueError(f'elec_theta: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    @property
    def PWM(self) -> uavcan.primitive.scalar.Integer32_1_0:
        """
        uavcan.primitive.scalar.Integer32.1.0 PWM
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._PWM

    @PWM.setter
    def PWM(self, x: uavcan.primitive.scalar.Integer32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Integer32_1_0):
            self._PWM = x
        else:
            raise ValueError(f'PWM: expected uavcan.primitive.scalar.Integer32_1_0 got {type(x).__name__}')

    @property
    def I_A(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 I_A
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._I_A

    @I_A.setter
    def I_A(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._I_A = x
        else:
            raise ValueError(f'I_A: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    @property
    def I_B(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 I_B
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._I_B

    @I_B.setter
    def I_B(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._I_B = x
        else:
            raise ValueError(f'I_B: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    @property
    def I_C(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 I_C
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._I_C

    @I_C.setter
    def I_C(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._I_C = x
        else:
            raise ValueError(f'I_C: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    @property
    def velocity_report(self) -> voltbro.config.six_step.pid_report_1_0:
        """
        voltbro.config.six_step.pid_report.1.0 velocity_report
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._velocity_report

    @velocity_report.setter
    def velocity_report(self, x: voltbro.config.six_step.pid_report_1_0) -> None:
        if isinstance(x, voltbro.config.six_step.pid_report_1_0):
            self._velocity_report = x
        else:
            raise ValueError(f'velocity_report: expected voltbro.config.six_step.pid_report_1_0 got {type(x).__name__}')

    @property
    def current_report(self) -> voltbro.config.six_step.pid_report_1_0:
        """
        voltbro.config.six_step.pid_report.1.0 current_report
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._current_report

    @current_report.setter
    def current_report(self, x: voltbro.config.six_step.pid_report_1_0) -> None:
        if isinstance(x, voltbro.config.six_step.pid_report_1_0):
            self._current_report = x
        else:
            raise ValueError(f'current_report: expected voltbro.config.six_step.pid_report_1_0 got {type(x).__name__}')

    # noinspection PyProtectedMember
    def _serialize_(self, _ser_: _Serializer_) -> None:
        assert _ser_.current_bit_length % 8 == 0, 'Serializer is not aligned'
        _base_offset_ = _ser_.current_bit_length
        _ser_.pad_to_alignment(8)
        self.is_on._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.is_stalling._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.target_speed._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.speed._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.elec_theta._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.PWM._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.I_A._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.I_B._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.I_C._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        # Delimited serialization of voltbro.config.six_step.pid_report.1.0, fixed bit length 128 (16 bytes)
        _ser_.add_aligned_u32(16)  # Delimiter header is constant in this case.
        _ser_base_offset_ = _ser_.current_bit_length
        self.velocity_report._serialize_(_ser_)
        assert _ser_.current_bit_length - _ser_base_offset_ == 128
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        # Delimited serialization of voltbro.config.six_step.pid_report.1.0, fixed bit length 128 (16 bytes)
        _ser_.add_aligned_u32(16)  # Delimiter header is constant in this case.
        _ser_base_offset_ = _ser_.current_bit_length
        self.current_report._serialize_(_ser_)
        assert _ser_.current_bit_length - _ser_base_offset_ == 128
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        assert 304 <= (_ser_.current_bit_length - _base_offset_) <= 560, \
            'Bad serialization of voltbro.config.six_step.log_item.1.0'

    # noinspection PyProtectedMember
    @staticmethod
    def _deserialize_(_des_: _Deserializer_) -> log_item_1_0:
        assert _des_.consumed_bit_length % 8 == 0, 'Deserializer is not aligned'
        _base_offset_ = _des_.consumed_bit_length
        # Temporary _f0_ holds the value of "is_on"
        _des_.pad_to_alignment(8)
        _f0_ = uavcan.primitive.scalar.Integer8_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f1_ holds the value of "is_stalling"
        _des_.pad_to_alignment(8)
        _f1_ = uavcan.primitive.scalar.Integer8_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f2_ holds the value of "target_speed"
        _des_.pad_to_alignment(8)
        _f2_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f3_ holds the value of "speed"
        _des_.pad_to_alignment(8)
        _f3_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f4_ holds the value of "elec_theta"
        _des_.pad_to_alignment(8)
        _f4_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f5_ holds the value of "PWM"
        _des_.pad_to_alignment(8)
        _f5_ = uavcan.primitive.scalar.Integer32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f6_ holds the value of "I_A"
        _des_.pad_to_alignment(8)
        _f6_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f7_ holds the value of "I_B"
        _des_.pad_to_alignment(8)
        _f7_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f8_ holds the value of "I_C"
        _des_.pad_to_alignment(8)
        _f8_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f9_ holds the value of "velocity_report"
        _des_.pad_to_alignment(8)
        # Delimited deserialization of voltbro.config.six_step.pid_report.1.0, extent 128
        _dh_ = _des_.fetch_aligned_u32()  # Read the delimiter header.
        if _dh_ * 8 > _des_.remaining_bit_length:
            raise _des_.FormatError(f'Delimiter header specifies {_dh_ * 8} bits, '
                                    f'but the remaining length is only {_des_.remaining_bit_length} bits')
        _nested_ = _des_.fork_bytes(_dh_)
        _des_.skip_bits(_dh_ * 8)
        _f9_ = voltbro.config.six_step.pid_report_1_0._deserialize_(_nested_)
        del _nested_
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f10_ holds the value of "current_report"
        _des_.pad_to_alignment(8)
        # Delimited deserialization of voltbro.config.six_step.pid_report.1.0, extent 128
        _dh_ = _des_.fetch_aligned_u32()  # Read the delimiter header.
        if _dh_ * 8 > _des_.remaining_bit_length:
            raise _des_.FormatError(f'Delimiter header specifies {_dh_ * 8} bits, '
                                    f'but the remaining length is only {_des_.remaining_bit_length} bits')
        _nested_ = _des_.fork_bytes(_dh_)
        _des_.skip_bits(_dh_ * 8)
        _f10_ = voltbro.config.six_step.pid_report_1_0._deserialize_(_nested_)
        del _nested_
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        self = log_item_1_0(
            is_on=_f0_,
            is_stalling=_f1_,
            target_speed=_f2_,
            speed=_f3_,
            elec_theta=_f4_,
            PWM=_f5_,
            I_A=_f6_,
            I_B=_f7_,
            I_C=_f8_,
            velocity_report=_f9_,
            current_report=_f10_)
        _des_.pad_to_alignment(8)
        assert 304 <= (_des_.consumed_bit_length - _base_offset_) <= 560, \
            'Bad deserialization of voltbro.config.six_step.log_item.1.0'
        assert isinstance(self, log_item_1_0)
        return self

    def __repr__(self) -> str:
        _o_0_ = ', '.join([
            'is_on=%s' % self.is_on,
            'is_stalling=%s' % self.is_stalling,
            'target_speed=%s' % self.target_speed,
            'speed=%s' % self.speed,
            'elec_theta=%s' % self.elec_theta,
            'PWM=%s' % self.PWM,
            'I_A=%s' % self.I_A,
            'I_B=%s' % self.I_B,
            'I_C=%s' % self.I_C,
            'velocity_report=%s' % self.velocity_report,
            'current_report=%s' % self.current_report,
        ])
        return f'voltbro.config.six_step.log_item.1.0({_o_0_})'

    _EXTENT_BYTES_ = 70

    # The big, scary blog of opaque data below contains a serialized PyDSDL object with the metadata of the
    # DSDL type this class is generated from. It is needed for reflection and runtime introspection.
    # Eventually we should replace this with ad-hoc constants such that no blob is needed and the generated code
    # is not dependent on PyDSDL.
    _MODEL_: _pydsdl_.DelimitedType = _restore_constant_(
        'ABzY8OyLG*0{`t-OKjX!6rJ>wr0FlEP(VmD1ys`C;J=^w76L_ONEoG1KoHCu`*p4dk8RnWC_zGMkw6p-1l259AhBRcMMVW-fl$SY'
        '9o=;aLVUJJNUXVU?D=GzNd*;%1(6az-+SME_jit8PH%eu;&r9=PqH^ywj<jun-L2g>N+pen#;<j<u?OAa$=Sy+aG4G)5N9y<Z{5$'
        'bLoLI=_~1UGHN=W$HEj!$@UX**s|hQ$X1(dHa*%z7gLMA8`na=Z28_Pr(TYnB{PayP<H*giJqHjGHx!iFmim4`x`r%ug|5GkxJ?H'
        '^h}y;HEA4&POTNQC|yA3Ln7~5rOD=F4s-2ZI1{E#V`|1cMi|S4Fp*5P=%PivauD(~okdoTEb7v*eB6s!orNTYZLp&g2<%!&;f8$k'
        'O~az-`bv{sC!D&+?A+Pv0&O*GPHei&tH+R5#xX*RMzPuSZH#bVvZvZ{!?%@ZP&dm}SeDXj>3o`8k7cBuO+&li(HpHArnPyIx-C3^'
        'lWF@_nv~L3jV8PMs%X}hv;Mr2!V%aTVKg=iLS|9yb$T}0VV-i9m~94r7@Llr&Lz9d$Zv%fLxs!CfZ|!n7=O4<ElsvmablLLTzU?3'
        '8@UV52A}tTL*S&zHfuQm>K@_ckJ99ppjC4n%M4k)<?<7I%|Y#G))REvSIN<=FIdt}XUbw(DD(QKF)zn%6KI5Sh0Ke2Llzy2rDvfG'
        'cVQK3F8bZyX;-gYXc!N^W3&%B@#F3H6Zo1pEZJ!K0gm%9C(&}V=3`PA$t-L7I1DjP@e=_HY3zsn(6PY5$%IO>vr28-@#+O5sB-OG'
        '%f*bB2es%S>+ZYhbY*-csv&-Y%n^=JtA^<|9gj~GF6a_pjPa?$O7q<lCR+-q*M`_!s}|L=fpH>};}6S)@7)oHn^CtFbvSCS%TFj+'
        '^ssd6Ff7e8>dMkONWyX7a?JH<yb2+(zcK}P!XY^P21ebab<c@gh=63vsgPQ{y4ck5WBD4lC3nh-AQ4GcG)*R|M2Kn-QPc%RB(g#j'
        'Rn!bgRTLrR0O|({loRcdC3NI2PlX%@5l#_D^OTcKTXgr-b4Nr?*L6wM4o@BGEEhPkKtL3|49y6nvmnob0{S!%FyL;u2kwRY;C^@@'
        '*_l;g_0+;ldqUx1c(k1}JO;Df5t#3;PG-tydB>-)<M14BG(3;>S-{^HQ1@c5m)l|*))|5zV2I^H!|FWkb#xQVWtJHOi|KmZ-ggz|'
        'T)*MB7h1f!9sJm=t{=h&>(=$U?s@-0X*X7%o?W|Y8;_e04%Iv~R5Mf5Y{*xe`-*+ru#afq*vO)sr3=-b{_7ysl=XTRbbEF*?-@j}'
        '_>4MIDv7$GYlbW$$}~w9Rb3#WqDs0V$-2w|D65)o5J4C4sY)tQ6vNOEorb6@x<&-PN74kH2nb+RCb}l5#86eD$&#*VL?@yys*0d+'
        'sjP~MOe94^_lhiuvZN7W;HyZ`m?(Gz{tB`x>m2e#6Yw-qP<2gJMdSxT*Ca(%3_&minHaK!n-xLF!-y&%O{k(Q8|a&#CQGu0R+1zc'
        '3K2Dx2t-v48BdjD%obC`)QO}?NHU6|WAYfRjM3^M5e(6g41*9+QU#1%kTj7fqNEV?tC6)>NVqx+Ne2(AGEz|c+2fz@;M=S7kEUER'
        'NRIY{<S2sV=paaTw4)8FYqBuJ?^a>Lzd@tX2yaHz&6SL&Fb&_sCHN73XswY)GL}OCzkeABko!9@z;A932!AWX*}f0&4Ku#S??K+0'
        'U+`p0_5sIx2EU4~>Fvsp0>Kp)ap`H4p6*FI#;e6yHQ_5n<kuOVFToF2FBJR)Kf^EZYhUvC{bx&<Pn=dxCwuUsm)+?@6BxDekzI=S'
        'CA**UqM`TIoT?90M)=rfguP}_f6qjJ@Q4fS+F$1z=c!j`^DJPoGwiPPQg(%z92i1=?P-^+)G1kIoXblqQM-0onK^}jR8;0;x^t1a'
        'zU9QrT>>4P(=YijTdlBtrCeo{w}HYQu%QNT0)w~VEm(nf;1hTcK85q}8C-zR;bP-<q(}Iou^%6eJMho&o+Z0F{}Nr}2={m3f6M=p'
        '<y;r*<2ak+Jqno1CUz2~lSN_}5Wkuk3n&$e#slKqxQfzh(Re_d8wV&2ipB%tk2B*Hl&%zw2gJGO3n;x%G#(J=#+OlgxoA8f&W(RV'
        '>2F2j)#Ao>L}1%~c5(z3W>-cEI|_=vJpxw=8;W}d6*+frMed%=xO)%M1Vf5=2kGHFQo}{$1WpJ42$M9XQQb!!jg9z@@ijK_y`%W2'
        'pVJ!?-0`&%dgG?9+r}-ptQ=d1v-4UFA??}fhP7F{{3rLgIc&ZX000'
    )
    assert isinstance(_MODEL_, _pydsdl_.DelimitedType)
