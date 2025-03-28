# AUTOGENERATED, DO NOT EDIT.
#
# Source file:
# /home/pi/cyphal-types/voltbro/config/dc/pid_config.1.0.dsdl
#
# Generated at:  2024-03-29 15:42:04.135042 UTC
# Is deprecated: no
# Fixed port ID: None
# Full name:     voltbro.config.dc.pid_config
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

if _NSAPIV_[0] != 1:
    raise RuntimeError(
        f"Incompatible Nunavut support API version: support { _NSAPIV_ }, package (1, 0, 0)"
    )

def _restore_constant_(encoded_string: str) -> object:
    import pickle, gzip, base64
    return pickle.loads(gzip.decompress(base64.b85decode(encoded_string)))

# noinspection PyUnresolvedReferences, PyPep8, PyPep8Naming, SpellCheckingInspection, DuplicatedCode
class pid_config_1_0:
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
                 multiplier:         None | uavcan.primitive.scalar.Real32_1_0 = None,
                 p_gain:             None | uavcan.primitive.scalar.Real32_1_0 = None,
                 i_gain:             None | uavcan.primitive.scalar.Real32_1_0 = None,
                 d_gain:             None | uavcan.primitive.scalar.Real32_1_0 = None,
                 integral_error_lim: None | uavcan.primitive.scalar.Real32_1_0 = None,
                 tolerance:          None | uavcan.primitive.scalar.Real32_1_0 = None) -> None:
        """
        voltbro.config.dc.pid_config.1.0
        Raises ValueError if any of the primitive values are outside the permitted range, regardless of the cast mode.
        :param multiplier:         uavcan.primitive.scalar.Real32.1.0 multiplier
        :param p_gain:             uavcan.primitive.scalar.Real32.1.0 p_gain
        :param i_gain:             uavcan.primitive.scalar.Real32.1.0 i_gain
        :param d_gain:             uavcan.primitive.scalar.Real32.1.0 d_gain
        :param integral_error_lim: uavcan.primitive.scalar.Real32.1.0 integral_error_lim
        :param tolerance:          uavcan.primitive.scalar.Real32.1.0 tolerance
        """
        self._multiplier:         uavcan.primitive.scalar.Real32_1_0
        self._p_gain:             uavcan.primitive.scalar.Real32_1_0
        self._i_gain:             uavcan.primitive.scalar.Real32_1_0
        self._d_gain:             uavcan.primitive.scalar.Real32_1_0
        self._integral_error_lim: uavcan.primitive.scalar.Real32_1_0
        self._tolerance:          uavcan.primitive.scalar.Real32_1_0

        if multiplier is None:
            self.multiplier = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(multiplier, uavcan.primitive.scalar.Real32_1_0):
            self.multiplier = multiplier
        else:
            raise ValueError(f'multiplier: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(multiplier).__name__}')

        if p_gain is None:
            self.p_gain = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(p_gain, uavcan.primitive.scalar.Real32_1_0):
            self.p_gain = p_gain
        else:
            raise ValueError(f'p_gain: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(p_gain).__name__}')

        if i_gain is None:
            self.i_gain = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(i_gain, uavcan.primitive.scalar.Real32_1_0):
            self.i_gain = i_gain
        else:
            raise ValueError(f'i_gain: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(i_gain).__name__}')

        if d_gain is None:
            self.d_gain = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(d_gain, uavcan.primitive.scalar.Real32_1_0):
            self.d_gain = d_gain
        else:
            raise ValueError(f'd_gain: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(d_gain).__name__}')

        if integral_error_lim is None:
            self.integral_error_lim = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(integral_error_lim, uavcan.primitive.scalar.Real32_1_0):
            self.integral_error_lim = integral_error_lim
        else:
            raise ValueError(f'integral_error_lim: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(integral_error_lim).__name__}')

        if tolerance is None:
            self.tolerance = uavcan.primitive.scalar.Real32_1_0()
        elif isinstance(tolerance, uavcan.primitive.scalar.Real32_1_0):
            self.tolerance = tolerance
        else:
            raise ValueError(f'tolerance: expected uavcan.primitive.scalar.Real32_1_0 '
                             f'got {type(tolerance).__name__}')

    @property
    def multiplier(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 multiplier
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._multiplier

    @multiplier.setter
    def multiplier(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._multiplier = x
        else:
            raise ValueError(f'multiplier: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    @property
    def p_gain(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 p_gain
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._p_gain

    @p_gain.setter
    def p_gain(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._p_gain = x
        else:
            raise ValueError(f'p_gain: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    @property
    def i_gain(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 i_gain
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._i_gain

    @i_gain.setter
    def i_gain(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._i_gain = x
        else:
            raise ValueError(f'i_gain: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    @property
    def d_gain(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 d_gain
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._d_gain

    @d_gain.setter
    def d_gain(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._d_gain = x
        else:
            raise ValueError(f'd_gain: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    @property
    def integral_error_lim(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 integral_error_lim
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._integral_error_lim

    @integral_error_lim.setter
    def integral_error_lim(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._integral_error_lim = x
        else:
            raise ValueError(f'integral_error_lim: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    @property
    def tolerance(self) -> uavcan.primitive.scalar.Real32_1_0:
        """
        uavcan.primitive.scalar.Real32.1.0 tolerance
        The setter raises ValueError if the supplied value exceeds the valid range or otherwise inapplicable.
        """
        return self._tolerance

    @tolerance.setter
    def tolerance(self, x: uavcan.primitive.scalar.Real32_1_0) -> None:
        if isinstance(x, uavcan.primitive.scalar.Real32_1_0):
            self._tolerance = x
        else:
            raise ValueError(f'tolerance: expected uavcan.primitive.scalar.Real32_1_0 got {type(x).__name__}')

    # noinspection PyProtectedMember
    def _serialize_(self, _ser_: _Serializer_) -> None:
        assert _ser_.current_bit_length % 8 == 0, 'Serializer is not aligned'
        _base_offset_ = _ser_.current_bit_length
        _ser_.pad_to_alignment(8)
        self.multiplier._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.p_gain._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.i_gain._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.d_gain._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.integral_error_lim._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        self.tolerance._serialize_(_ser_)
        assert _ser_.current_bit_length % 8 == 0, 'Nested object alignment error'
        _ser_.pad_to_alignment(8)
        assert 192 <= (_ser_.current_bit_length - _base_offset_) <= 192, \
            'Bad serialization of voltbro.config.dc.pid_config.1.0'

    # noinspection PyProtectedMember
    @staticmethod
    def _deserialize_(_des_: _Deserializer_) -> pid_config_1_0:
        assert _des_.consumed_bit_length % 8 == 0, 'Deserializer is not aligned'
        _base_offset_ = _des_.consumed_bit_length
        # Temporary _f0_ holds the value of "multiplier"
        _des_.pad_to_alignment(8)
        _f0_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f1_ holds the value of "p_gain"
        _des_.pad_to_alignment(8)
        _f1_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f2_ holds the value of "i_gain"
        _des_.pad_to_alignment(8)
        _f2_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f3_ holds the value of "d_gain"
        _des_.pad_to_alignment(8)
        _f3_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f4_ holds the value of "integral_error_lim"
        _des_.pad_to_alignment(8)
        _f4_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        # Temporary _f5_ holds the value of "tolerance"
        _des_.pad_to_alignment(8)
        _f5_ = uavcan.primitive.scalar.Real32_1_0._deserialize_(_des_)
        assert _des_.consumed_bit_length % 8 == 0, 'Nested object alignment error'
        self = pid_config_1_0(
            multiplier=_f0_,
            p_gain=_f1_,
            i_gain=_f2_,
            d_gain=_f3_,
            integral_error_lim=_f4_,
            tolerance=_f5_)
        _des_.pad_to_alignment(8)
        assert 192 <= (_des_.consumed_bit_length - _base_offset_) <= 192, \
            'Bad deserialization of voltbro.config.dc.pid_config.1.0'
        assert isinstance(self, pid_config_1_0)
        return self

    def __repr__(self) -> str:
        _o_0_ = ', '.join([
            'multiplier=%s' % self.multiplier,
            'p_gain=%s' % self.p_gain,
            'i_gain=%s' % self.i_gain,
            'd_gain=%s' % self.d_gain,
            'integral_error_lim=%s' % self.integral_error_lim,
            'tolerance=%s' % self.tolerance,
        ])
        return f'voltbro.config.dc.pid_config.1.0({_o_0_})'

    _EXTENT_BYTES_ = 24

    # The big, scary blog of opaque data below contains a serialized PyDSDL object with the metadata of the
    # DSDL type this class is generated from. It is needed for reflection and runtime introspection.
    # Eventually we should replace this with ad-hoc constants such that no blob is needed and the generated code
    # is not dependent on PyDSDL.
    _MODEL_: _pydsdl_.DelimitedType = _restore_constant_(
        'ABzY8OyLG*0{_ibZEqVz5OzxQ-WN(}C<0OUOPh%Hh#lK;iUgsBNZ>Y&5)vU$*zVnKJX_Xxr@cMPE<{Q{fEsBfB4zpBpOT-#Phr+)'
        'C$^Im@r_84_hx3Foqgt++5O9Z%}?e!f97`FW(kuXNw^YJikH-v+#^BMh@(VkZq3!NxfBgtvhUk5w?}sVmEE&*W{ik1<jO+MT;0?v'
        '4RosbV8|#5X#*iXZbnl3D)NFT+!b|?1zs!|$-b<aAWg0k5rqzGe7n0ovbB*~?hpIQnrT9{R>Ds;PwWl?Uy$`llr^KPf=hOq$`oN#'
        'Q=(mdFrFD<%6yX2W<WzPR_=<T$-N|?k}7Y5Q@L2Qa0RaQ(tsN~7H)R8-nuA<6C`UUS7k)?ASKhp7n(>O)-~8#9`g!lqDdoSnAaUM'
        'yWZRGwt0`y_td$NjNI>b!<zR=qG`yeV*Nm`5vM`xq0lLF2k$BxXe2Q~a#y=+#4z_3OAEznaj{gXELO|KYPGyvEfkg(N`>lTsah%*'
        'D$B)kskC6-S*ec%jW(%F5%Mx&QD9Bwn--$Uc3$yV@q~xk-8wHq&FfgVhxwl#6e>$gOT|Lve*V+matE=fndz${@!Od;_AHd(c7nVx'
        'hZs=QnLRMq$gXH{M&d|mBA8t>H%JntD&XjloWvAwZN}Y?6h7W~9o5=ech)0hdE_(97$T>=OR=!#O3;ph$_K9e$jL`>>Prz2#p|hb'
        'C!UG}y<?ds5VWtBW0@|jxt=M*D|ic@tH0H*EHeaXf>|kK#0jD)0&WkWY;uk|Un1<?o?cGthLL*s8<Tw`^pno_CVqC6W8NWAY)$?m'
        'kfh!4BfJx)@{~6sRHLi;vzRNYBh}xv0SIgi*fiJIDPtn6p9z8=oOdmic;WW27ZEZ^{AuTE`NPD=_Kh<jTboFIylz8;PTIJjEw>nV'
        '@<yWF?kU1sF%3H%zLxrl-%;6ABb8djQn-0ih8h}A8Vx?gfU%g=DZ-xb6mN5~w>aPH2v<=_xKfcKIOxs~oYav-9zv{Y=I5~FyaHxH'
        '!V8F?0SaPBA%PY&Q&>KLhimXTd@%=K!U}u^kIZ;?3L=HQ$#;plf}PI7K~=$b2ktO<0?#_hzy@rcROrW()mC5CZa$7e*F)jC<u{XA'
        '^--m6jQ$JSL3XJ5e@1RY;N%9ZmIsdiyBG|pDA?wSv#y|Z<<!~>9x``>pQH8p+2A2_H+Ubd`)7lP%-!HOXnk`wc*xuhK1S>D+2BEQ'
        ')-o`SPt&{ZNt#0mzCiiKD81%?mnFP}U*R{{>k}L{EN`KPw5=VRSscvS`>zOa%J3(9MDLiHE$riZ$l15q_#b%b;4T*B9?aK9+*<<o'
        'Ut`wi3c0_~KWA&&n=J#{{$PWKb-ux4uEj;e?U!uk+`<k0SQCdG)>tjqqp~*P?773F!)Zpp6Vl&joo=i557kdE0W=8!00'
    )
    assert isinstance(_MODEL_, _pydsdl_.DelimitedType)
