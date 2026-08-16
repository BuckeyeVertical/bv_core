"""Framework-neutral metadata carried with a captured vision frame."""

from dataclasses import dataclass
from typing import Any, Mapping


@dataclass(frozen=True)
class SimulationFrameMetadata:
    stream_id: str
    sequence: int
    time_ns: int

    @classmethod
    def from_mapping(cls, metadata: Mapping[str, Any]):
        fields = ("sim_stream_id", "sim_sequence", "sim_time_ns")
        values = tuple(metadata.get(field) for field in fields)
        if all(value is None for value in values):
            return None
        stream_id, sequence, time_ns = values
        if sequence is None or time_ns is None:
            raise ValueError("simulation frame metadata is incomplete")

        if stream_id is None:
            stream_id = ""
        elif not isinstance(stream_id, str) or not stream_id.strip():
            raise ValueError(
                "simulation frame metadata has an invalid stream ID"
            )
        for name, value in (("sequence", sequence), ("time", time_ns)):
            if (
                not isinstance(value, int)
                or isinstance(value, bool)
                or value < 0
            ):
                raise ValueError(
                    f"simulation frame metadata has an invalid {name}"
                )
        return cls(stream_id, sequence, time_ns)


def apply_simulation_metadata(
    message,
    metadata: SimulationFrameMetadata | None,
) -> None:
    if metadata is None:
        return
    message.sim_stream_id = metadata.stream_id
    message.sim_sequence = metadata.sequence
    message.sim_time_ns = metadata.time_ns
