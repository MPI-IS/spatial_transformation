# SPDX-License-Identifier: BSD-3-Clause
"""Represent 3d-transformations."""
from __future__ import annotations

import typing

import numpy as np
from scipy.spatial.transform import Rotation

from . import cpp

if typing.TYPE_CHECKING:
    import numpy.typing as npt


class Transformation:
    """Represents a 3d-transformation consisting of rotation and translation.

    The transformation consists of a rotation R and a translation T with the
    rotation being applied first.  So the transformed version v' of a vector v is
    computed as ``v' = R*v + T``.
    """

    rotation: Rotation
    """Rotation part of the transformation."""
    translation: np.ndarray
    """Translation part of the transformation."""

    def __init__(
        self,
        rotation: typing.Optional[typing.Union[Rotation, npt.ArrayLike]] = None,
        translation: typing.Optional[npt.ArrayLike] = None,
    ) -> None:
        """
        Args:
            rotation: The rotation part of the transformation either as a
                :class:`scipy.spatial.transform.Rotation` instance or a quaternion in
                (x, y, z, w) format.  If not set, no rotation is done.
            translation: The translation part of the transformation.  If not set, no
                translation is done.
        """
        if rotation is None:
            self.rotation = Rotation.identity()
        elif isinstance(rotation, Rotation):
            self.rotation = rotation
        else:
            # assume rotation is given as quaternion
            self.rotation = Rotation.from_quat(rotation)

        if translation is None:
            self.translation = np.zeros(3)
        else:
            self.translation = np.asarray(translation)

    @classmethod
    def identity(cls) -> Transformation:
        """Create a identity transformation.

        This is equivalent to using the constructor with default arguments but exists
        for clarity and consistency with the Rotation class.
        """
        return cls()

    @classmethod
    def from_cpp(cls, cpp_tf: cpp.Transformation) -> Transformation:
        """Construct from :class:`cpp.Transformation`."""
        return cls(rotation=cpp_tf.get_rotation(), translation=cpp_tf.translation)

    def __mul__(self, other: Transformation) -> Transformation:
        """Compose this transformation with the other."""
        r_new = self.rotation * other.rotation
        t_new = self.translation + self.rotation.apply(other.translation)
        return Transformation(r_new, t_new)

    def inv(self) -> Transformation:
        """Invert the transformation."""
        inv_rot = self.rotation.inv()
        inv_trans = -inv_rot.apply(self.translation)
        return Transformation(inv_rot, inv_trans)

    def apply(self, vector: npt.ArrayLike) -> npt.NDArray:
        """Apply the transformation to the given vector."""
        return self.rotation.apply(vector) + self.translation

    def as_matrix(self) -> npt.NDArray:
        """Convert to homogeneous transformation matrix."""
        mat = np.eye(4)
        mat[:3, :3] = self.rotation.as_matrix()
        mat[:3, 3] = self.translation
        return mat

    def as_cpp(self) -> cpp.Transformation:
        """Convert to :class:`cpp.Transformation`."""
        cpp_tf = cpp.Transformation()
        cpp_tf.translation = self.translation
        cpp_tf.set_rotation(self.rotation.as_quat())
        return cpp_tf

    def __repr__(self) -> str:
        """Convert to string representation."""
        return "Transformation(rotation={}, translation={})".format(
            self.rotation.as_quat(), self.translation
        )
