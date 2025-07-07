# models.py
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any, Union
from datetime import datetime


class Response(BaseModel):
    msg: str
    timestamp: Optional[str] = Field(default_factory=lambda: datetime.now().isoformat())


class TelemetryStatus(BaseModel):
    status: str
    timestamp: str
    data: Dict[str, Any]


class OdometryData(BaseModel):
    position_x: float
    position_y: float
    position_z: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    linear_velocity_x: float
    linear_velocity_y: float
    linear_velocity_z: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    timestamp: str


class ImuData(BaseModel):
    acceleration_x: float
    acceleration_y: float
    acceleration_z: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    timestamp: str


class GpsData(BaseModel):
    latitude: float
    longitude: float
    altitude: float
    accuracy: float
    timestamp: str


class ImageInfo(BaseModel):
    width: int
    height: int
    encoding: str
    timestamp: str
    frame_id: str


class NodeStatus(BaseModel):
    node_name: str
    namespace: str
    is_running: bool
    message_count: Optional[int] = 0


class PublishRequest(BaseModel):
    message: str = "Hello World"


class CmdVelRequest(BaseModel):
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0


class ServiceRequest(BaseModel):
    a: int
    b: int


class ParameterUpdate(BaseModel):
    value: Union[str, int, float, bool]


class MessageData(BaseModel):
    type: str
    data: Any
    timestamp: str


class ServiceCallInfo(BaseModel):
    call_id: int
    input: Dict[str, Any]
    result: Any
    timestamp: str


class SystemStatus(BaseModel):
    status: str
    message: str
    nodes_count: int
    is_running: bool
    uptime: Optional[str] = None
