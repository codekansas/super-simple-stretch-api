import array as arr
import struct
from abc import ABC
from dataclasses import Field, dataclass, field, fields
from typing import TYPE_CHECKING, Literal, Sequence, Type, TypeVar, cast, overload

if TYPE_CHECKING:
    from _typeshed import WriteableBuffer

Tb = TypeVar("Tb")
Tr = TypeVar("Tr", arr.array, bytes, bytearray)


def pack_string_t(s: "WriteableBuffer", sidx: int, x: str) -> int:
    xb = x.encode("utf-8")
    struct.pack_into(str(len(xb)) + "s", s, sidx, xb)
    return sidx + len(x)


def unpack_string_t(s: Tr, n: int) -> tuple[str, Tr]:
    return (struct.unpack(str(n) + "s", s[:n])[0].strip(b"\x00")).decode("utf-8"), s[n:]


def pack_int32_t(s: "WriteableBuffer", sidx: int, x: int) -> int:
    struct.pack_into("i", s, sidx, x)
    return sidx + 4


def unpack_int32_t(s: Tr) -> tuple[int, Tr]:
    return struct.unpack("i", s[:4])[0], s[4:]


def pack_uint32_t(s: "WriteableBuffer", sidx: int, x: int) -> int:
    struct.pack_into("I", s, sidx, x)
    return sidx + 4


def unpack_uint32_t(s: Tr) -> tuple[int, Tr]:
    return struct.unpack("I", s[:4])[0], s[4:]


def pack_int64_t(s: "WriteableBuffer", sidx: int, x: int) -> int:
    struct.pack_into("q", s, sidx, x)
    return sidx + 8


def unpack_int64_t(s: Tr) -> tuple[int, Tr]:
    return struct.unpack("q", s[:8])[0], s[8:]


def pack_uint64_t(s: "WriteableBuffer", sidx: int, x: int) -> int:
    struct.pack_into("Q", s, sidx, x)
    return sidx + 8


def unpack_uint64_t(s: Tr) -> tuple[int, Tr]:
    return struct.unpack("Q", s[:8])[0], s[8:]


def pack_int16_t(s: "WriteableBuffer", sidx: int, x: int) -> int:
    struct.pack_into("h", s, sidx, x)
    return sidx + 2


def unpack_int16_t(s: Tr) -> tuple[int, Tr]:
    return struct.unpack("h", s[:2])[0], s[2:]


def pack_uint16_t(s: "WriteableBuffer", sidx: int, x: int) -> int:
    struct.pack_into("H", s, sidx, x)
    return sidx + 2


def unpack_uint16_t(s: Tr) -> tuple[int, Tr]:
    return struct.unpack("H", s[:2])[0], s[2:]


def pack_int8_t(s: "WriteableBuffer", sidx: int, x: int) -> int:
    struct.pack_into("b", s, sidx, x)
    return sidx + 1


def unpack_int8_t(s: Tr) -> tuple[int, Tr]:
    return struct.unpack("b", s[:1])[0], s[1:]


def pack_uint8_t(s: "WriteableBuffer", sidx: int, x: int) -> int:
    struct.pack_into("B", s, sidx, x)
    return sidx + 1


def unpack_uint8_t(s: Tr) -> tuple[int, Tr]:
    return struct.unpack("B", s[:1])[0], s[1:]


def pack_float_t(s: "WriteableBuffer", sidx: int, x: float) -> int:
    struct.pack_into("f", s, sidx, x)
    return sidx + 4


def unpack_float_t(s: Tr) -> tuple[float, Tr]:
    return struct.unpack("f", s[:4])[0], s[4:]


def pack_double_t(s: "WriteableBuffer", sidx: int, x: float) -> int:
    struct.pack_into("d", s, sidx, x)
    return sidx + 8


def unpack_double_t(s: Tr) -> tuple[float, Tr]:
    return struct.unpack("d", s[:8])[0], s[8:]


StringDtype = Literal["string"]
FloatDtype = Literal["float", "double"]
IntDtype = Literal["int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64"]
Dtype = StringDtype | FloatDtype | IntDtype


@overload
def byte_size(dtype: StringDtype, n: int) -> int:
    ...


@overload
def byte_size(dtype: FloatDtype | IntDtype) -> int:
    ...


def byte_size(dtype: Dtype, n: int | None = None) -> int:
    match dtype:
        case "string":
            assert n is not None, "Expected `n` argument for string dtype"
            return n
        case "float":
            return 4
        case "double":
            return 8
        case "int8":
            return 1
        case "uint8":
            return 1
        case "int16":
            return 2
        case "uint16":
            return 2
        case "int32":
            return 4
        case "uint32":
            return 4
        case "int64":
            return 8
        case "uint64":
            return 8
        case _:
            raise NotImplementedError(f"Unsupported dtype: {dtype}")


def total_byte_size(dtypes: Sequence[IntDtype | FloatDtype | tuple[StringDtype, int]]) -> int:
    s = 0
    for dtype in dtypes:
        if isinstance(dtype, tuple):
            s += byte_size(dtype[0], dtype[1])
        else:
            s += byte_size(dtype)
    return s


@overload
def pack(x: str, dtype: StringDtype, s: "WriteableBuffer", sidx: int) -> int:
    ...


@overload
def pack(x: float, dtype: FloatDtype, s: "WriteableBuffer", sidx: int) -> int:
    ...


@overload
def pack(x: int, dtype: IntDtype, s: "WriteableBuffer", sidx: int) -> int:
    ...


def pack(x: str | int | float, dtype: Dtype, s: "WriteableBuffer", sidx: int) -> int:
    match dtype:
        case "string":
            assert isinstance(x, str), f"Expected string, got {type(x)}"
            return pack_string_t(s, sidx, x)
        case "float":
            assert isinstance(x, float), f"Expected float, got {type(x)}"
            return pack_float_t(s, sidx, x)
        case "double":
            assert isinstance(x, float), f"Expected float, got {type(x)}"
            return pack_double_t(s, sidx, x)
        case "int8":
            assert isinstance(x, int), f"Expected int, got {type(x)}"
            return pack_int8_t(s, sidx, x)
        case "uint8":
            assert isinstance(x, int), f"Expected int, got {type(x)}"
            return pack_uint8_t(s, sidx, x)
        case "int16":
            assert isinstance(x, int), f"Expected int, got {type(x)}"
            return pack_int16_t(s, sidx, x)
        case "uint16":
            assert isinstance(x, int), f"Expected int, got {type(x)}"
            return pack_uint16_t(s, sidx, x)
        case "int32":
            assert isinstance(x, int), f"Expected int, got {type(x)}"
            return pack_int32_t(s, sidx, x)
        case "uint32":
            assert isinstance(x, int), f"Expected int, got {type(x)}"
            return pack_uint32_t(s, sidx, x)
        case "int64":
            assert isinstance(x, int), f"Expected int, got {type(x)}"
            return pack_int64_t(s, sidx, x)
        case "uint64":
            assert isinstance(x, int), f"Expected int, got {type(x)}"
            return pack_uint64_t(s, sidx, x)
        case _:
            raise NotImplementedError(f"Unsupported packing dtype: {dtype}")


def pack_many(
    inputs: Sequence[tuple[int, IntDtype] | tuple[float, FloatDtype] | tuple[str, StringDtype]],
    s: "WriteableBuffer",
    sidx: int,
) -> int:
    for x, dtype in inputs:
        sidx = pack(x, dtype, s, sidx)  # type: ignore
    return sidx


@overload
def unpack(dtype: StringDtype, s: Tr, n: int) -> tuple[str, Tr]:
    ...


@overload
def unpack(dtype: FloatDtype, s: Tr) -> tuple[float, Tr]:
    ...


@overload
def unpack(dtype: IntDtype, s: Tr) -> tuple[int, Tr]:
    ...


def unpack(dtype: Dtype, s: Tr, n: int | None = None) -> tuple[str | int | float, Tr]:
    match dtype:
        case "string":
            assert n is not None, "Expected `n` argument for unpacking strings"
            return unpack_string_t(s, n)
        case "float":
            return unpack_float_t(s)
        case "double":
            return unpack_double_t(s)
        case "int8":
            return unpack_int8_t(s)
        case "uint8":
            return unpack_uint8_t(s)
        case "int16":
            return unpack_int16_t(s)
        case "uint16":
            return unpack_uint16_t(s)
        case "int32":
            return unpack_int32_t(s)
        case "uint32":
            return unpack_uint32_t(s)
        case "int64":
            return unpack_int64_t(s)
        case "uint64":
            return unpack_uint64_t(s)
        case _:
            raise NotImplementedError(f"Unsupported unpacking type: {dtype}")


def unpack_many(
    s: Tr,
    dtypes: Sequence[IntDtype | FloatDtype | tuple[StringDtype, int]],
) -> tuple[Sequence[int | float | str], Tr]:
    outputs: list[int | float | str] = []
    for dtype in dtypes:
        if isinstance(dtype, tuple):
            string_dtype, length = dtype
            string_output, s = unpack(string_dtype, s, n=length)
            outputs.append(string_output)
        else:
            output, s = unpack(dtype, s)
            outputs.append(output)
    return outputs, s


@overload
def byte_field(dtype: IntDtype) -> int:
    ...


@overload
def byte_field(dtype: FloatDtype) -> int:
    ...


@overload
def byte_field(dtype: StringDtype, n: int) -> str:
    ...


def byte_field(dtype: IntDtype | FloatDtype | StringDtype, n: int | None = None) -> int | float | str:
    metadata: dict[str, Dtype | int] = {}
    default: int | float | str
    if n is None:
        assert dtype != "string", "Unexpected `n` argument for non-string dtype"
        metadata["dtype"] = dtype
        default = 0.0 if dtype in ("float", "double") else 0
    else:
        assert dtype == "string", "Expected `n` argument for string dtype"
        metadata["dtype"] = dtype
        metadata["length"] = n
        default = ""
    return field(default=default, metadata=metadata)


def flag_field(ref_field: str, i: int) -> bool:
    return field(default=False, metadata={"ref": ref_field, "mask": (1 << i)})


@dataclass
class Bytes(ABC):
    @classmethod
    def dtypes(cls) -> list[IntDtype | FloatDtype | tuple[StringDtype, int]]:
        dtypes: list[IntDtype | FloatDtype | tuple[StringDtype, int]] = []
        for f in fields(cls):
            if not hasattr(f, "metadata") or not f.metadata or "dtype" not in f.metadata:
                continue
            dtype = cast(Dtype, f.metadata["dtype"])
            if dtype == "string":
                dtypes.append((dtype, cast(int, f.metadata["length"])))
            else:
                dtypes.append(dtype)
        return dtypes

    def packable(self) -> Sequence[tuple[int, IntDtype] | tuple[float, FloatDtype] | tuple[str, StringDtype]]:
        values: list[tuple[int, IntDtype] | tuple[float, FloatDtype] | tuple[str, StringDtype]] = []

        # Separates the flags from the rest of the fields.
        byte_fields: list[Field] = []
        flag_fields: list[Field] = []
        for f in fields(self):
            if not hasattr(f, "metadata") or not f.metadata:
                continue
            if "ref" in f.metadata:
                flag_fields.append(f)
            elif "dtype" in f.metadata:
                if flag_fields:
                    raise ValueError("Flag fields must come before byte fields")
                byte_fields.append(f)

        # First, set all flags.
        for f in flag_fields:
            ref = cast(str, f.metadata["ref"])
            mask = cast(int, f.metadata["mask"])
            if not hasattr(self, ref):
                raise ValueError(f"Field {f.name} references missing field {ref}")
            value = getattr(self, f.name)
            if value:
                setattr(self, ref, getattr(self, ref) | mask)
            else:
                setattr(self, ref, getattr(self, ref) & ~mask)

        # Next, loop through the byte values and pack them.
        for f in byte_fields:
            dtype = cast(Dtype, f.metadata["dtype"])
            value = getattr(self, f.name)
            match dtype:
                case "string":
                    assert isinstance(value, str), f"Expected str, got {type(value)}"
                    values.append((value, dtype))
                case "float" | "double":
                    assert isinstance(value, float), f"Expected float, got {type(value)}"
                    values.append((value, dtype))
                case "int8" | "uint8" | "int16" | "uint16" | "int32" | "uint32" | "int64" | "uint64":
                    assert isinstance(value, int), f"Expected int, got {type(value)}"
                    values.append((value, dtype))
                case _:
                    raise NotImplementedError(f"Unsupported packing dtype: {dtype}")

        return values

    def total_bytes(cls) -> int:
        return total_byte_size(cls.dtypes())

    @classmethod
    def unpack(cls: Type[Tb], s: Tr) -> tuple[Tb, Tr]:
        values, s = unpack_many(s, cls.dtypes())
        return cls(*values), s

    def pack(self, s: "WriteableBuffer", sidx: int) -> int:
        return pack_many(self.packable(), s, sidx)

    def __post_init__(self) -> None:
        # Separates the flags from the rest of the fields.
        flag_fields: list[Field] = []
        for f in fields(self):
            if not hasattr(f, "metadata") or not f.metadata:
                pass
            if "ref" in f.metadata:
                flag_fields.append(f)
            elif "dtype" in f.metadata:
                if flag_fields:
                    raise ValueError("Flag fields must come before byte fields")

        # Sets the flags from their reference field.
        for f in flag_fields:
            ref = cast(str, f.metadata["ref"])
            mask = cast(int, f.metadata["mask"])
            if not hasattr(self, ref):
                raise ValueError(f"Field {f.name} references missing field {ref}")
            value = getattr(self, ref)
            setattr(self, f.name, (value & mask) > 0)
