Web UI (sovd_web_ui)
====================

sovd_web_ui is a lightweight web application for browsing SOVD entity trees.
It connects to the ros2_medkit gateway and visualizes the entity hierarchy.

.. contents:: Table of Contents
   :local:
   :depth: 2

Features
--------

- **Server Connection Dialog** — Enter the URL of your gateway
- **Entity Tree Sidebar** — Browse the hierarchical structure with lazy-loading
- **Entity Detail Panel** — View raw JSON details of any selected entity
- **Data/Operations/Configurations** — Virtual folders for each entity

Quick Start
-----------

Using Docker
~~~~~~~~~~~~

.. code-block:: bash

   # Pull from GitHub Container Registry
   docker pull ghcr.io/selfpatch/sovd_web_ui:latest
   docker run -p 8080:80 ghcr.io/selfpatch/sovd_web_ui:latest

Then open http://localhost:8080 in your browser.

From Source
~~~~~~~~~~~

.. code-block:: bash

   # Clone the repository
   git clone https://github.com/selfpatch/sovd_web_ui.git
   cd sovd_web_ui

   # Install dependencies
   npm install

   # Start development server (port 5173)
   npm run dev

   # Build for production
   npm run build

Connecting to ros2_medkit
-------------------------

1. Open the web UI in your browser
2. Enter the gateway URL (e.g., ``http://localhost:8080``)
3. Click **Connect**
4. Browse the entity tree in the left sidebar
5. Click on any entity to view its details

.. tip::

   If the gateway runs on a different host, ensure CORS is configured.
   See :doc:`/config/server` for CORS settings.

Docker Compose Example
----------------------

Run both gateway and web UI together:

.. code-block:: yaml

   # docker-compose.yml
   version: '3.8'
   services:
     gateway:
       image: ros:jazzy
       command: >
         bash -c "source /opt/ros/jazzy/setup.bash &&
                  ros2 launch ros2_medkit_gateway gateway.launch.py server_host:=0.0.0.0"
       ports:
         - "8080:8080"
       network_mode: host

     web_ui:
       image: ghcr.io/selfpatch/sovd_web_ui:latest
       ports:
         - "80:80"

Docker Image Tags
-----------------

Images are published to GitHub Container Registry:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Trigger
     - Image Tags
   * - Push/merge to ``main``
     - ``latest``, ``sha-<commit>``
   * - Git tag ``v1.2.3``
     - ``1.2.3``, ``1.2``, ``1``, ``sha-<commit>``

Tech Stack
----------

- **React 19** — UI framework
- **TypeScript** — Type safety
- **Vite** — Build tool
- **TailwindCSS 4** — Styling
- **shadcn/ui** — UI components
- **Zustand** — State management

Repository
----------

https://github.com/selfpatch/sovd_web_ui

See Also
--------

- :doc:`/getting_started` — Basic gateway setup
- :doc:`/config/server` — Configure CORS for web UI access
