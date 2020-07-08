Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-lps2x/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/lps2x/en/latest/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_LPS2x/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_LPS2x/actions
    :alt: Build Status

Library for the ST LPS2x family of barometric pressure sensors

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://circuitpython.org/downloads>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Register <https://github.com/adafruit/Adafruit_CircuitPython_Register>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_.

Installing from PyPI
=====================
On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-lps2x/>`_. To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-lps2x

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-lps2x

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .env
    source .env/bin/activate
    pip3 install adafruit-circuitpython-lps2x

Usage Example
=============

.. code-block:: python

    import time
    import board
    import busio
    import adafruit_lps2x

    i2c = busio.I2C(board.SCL, board.SDA)
    # uncomment and comment out the line after to use with the LPS22
    # lps = adafruit_lps2x.LPS22(i2c)
    lps = adafruit_lps2x.LPS25(i2c)
    while True:
        print("Pressure: %.2f hPa" % lps.pressure)
        print("Temperature: %.2f C" % lps.temperature)
        time.sleep(1)

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_LPS2x/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Documentation
=============

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.
