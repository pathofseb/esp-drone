#!/usr/bin/env python3
"""
FastAPI Server for ESP32 Drone PID Tuning
Provides REST API for the web frontend
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import socket
import struct
import uvicorn

app = FastAPI(title="ESP32 Drone PID Tuner API")

# Enable CORS for frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

UDP_PORT = 4210


class PIDParameters(BaseModel):
    droneIp: str
    PRateRoll: float
    PRatePitch: float
    PRateYaw: float
    IRateRoll: float
    IRatePitch: float
    IRateYaw: float
    DRateRoll: float
    DRatePitch: float
    DRateYaw: float
    PAngleRoll: float
    PAnglePitch: float
    IAngleRoll: float
    IAnglePitch: float
    DAngleRoll: float
    DAnglePitch: float


def pack_pid_parameters(params: PIDParameters):
    """Pack PID parameters into binary format matching C++ struct"""
    return struct.pack('14f',
        params.PRateRoll,
        params.PRatePitch,
        params.PRateYaw,
        params.IRateRoll,
        params.IRatePitch,
        params.IRateYaw,
        params.DRateRoll,
        params.DRatePitch,
        params.DRateYaw,
        params.PAngleRoll,
        params.PAnglePitch,
        params.IAngleRoll,
        params.IAnglePitch,
        params.DAngleRoll,
        params.DAnglePitch
    )


@app.post("/send")
async def send_parameters(params: PIDParameters):
    """Send PID parameters to the drone via UDP"""
    try:
        # Pack the parameters
        binary_data = pack_pid_parameters(params)

        # Send via UDP
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(binary_data, (params.droneIp, UDP_PORT))
        sock.close()

        return {
            'success': True,
            'message': f'Sent {len(binary_data)} bytes to {params.droneIp}:{UDP_PORT}'
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/health")
async def health():
    """Health check endpoint"""
    return {'status': 'ok'}


if __name__ == '__main__':
    print("🚀 Starting FastAPI Server...")
    print("📡 API available at http://localhost:8000")
    print("📚 Docs available at http://localhost:8000/docs")
    print("🌐 Open frontend/index.html in your browser")
    uvicorn.run(app, host="0.0.0.0", port=8000)
