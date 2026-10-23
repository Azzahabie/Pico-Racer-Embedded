"""Launch the amqtt broker through python.exe (not amqtt.exe).

    py tools\\run_broker.py

Listens on 0.0.0.0:1883.  Point config/broker_config.h at this laptop's LAN
address, and watch the car's records with tools/telemetry_decode.py.
Needs amqtt (pip install amqtt; 0.12.1 is the version verified for M6).

Why this exists: on Windows the firewall allow-rule is bound to
python.exe, so the broker must run as python.exe for the Pico to reach it
over the LAN.  `amqtt.exe` is a different executable and would not match.
`python -m amqtt` is unsupported (no __main__), hence this shim.
"""

from amqtt.scripts.broker_script import main

if __name__ == "__main__":
    main()