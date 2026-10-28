# Host Control Application

**A Python-based remote control and data acquisition system for an energy-efficient robotic vehicle.**

**Status:** In development

## Overview

This application is designed to remotely control a miniature electric vehicle powered by an ESP32 and collect experimental data for energy optimization research.

The objective is to minimize electrical energy consumption over a fixed travel distance while respecting a maximum completion time.

Control is organized around fixed-distance windows. At the end of each window, the ESP32 sends sensor measurements to the host, which processes observations and schedules commands for subsequent windows.

## Architecture

The application is designed around independent services communicating through Redis Streams.

- **UDP service:** Handles communication with the ESP32 and validates incoming and outgoing packets.
- **Controller service:** Processes observations and generates motor commands.
- **Episode logger:** Records measurements and experiment metadata in CSV and JSON formats.
- **Dashboard:** Provides real-time monitoring and visualization using Plotly Dash.
- **Simulator:** Optionally emulates communication and control behavior without the physical vehicle.
- **Redis:** Connects the host-side services through event streams and shared state.

## Control and Optimization

A modular, event-driven controller interface will support several control strategies:

- Manual control and predefined command profiles.
- Reinforcement learning policies.
- Dynamic programming and alternative optimization methods.

The goal is to compare these strategies using measurements collected under repeatable experimental conditions.

## Technology Stack

| Area | Technologies |
|---|---|
| Application | Python |
| Communication | Wi-Fi, UDP |
| Event bus | Redis Streams |
| Dashboard | Plotly Dash |
| Data logging | CSV, JSON |
| Deployment | Docker, Docker Compose |
| Embedded system | ESP32, C++, PlatformIO |

## Testing and Reproducibility

An optional simulator and automated test suite are planned to validate packet encoding, communication, command scheduling, safety behavior, and episode recording before testing with the physical vehicle.

The simulator does not model full motor or battery dynamics, so simulated measurements are not equivalent to real experimental results.

## Development Status

The application architecture and requirements are being specified, and implementation is in progress. Complete integration with the embedded system and validation on the physical vehicle remain future milestones.

See [`requirements.md`](requirements.md) for the detailed functional specification and [`tasks.md`](tasks.md) for the implementation roadmap.
