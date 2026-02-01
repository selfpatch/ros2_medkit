Web UI (sovd_web_ui)
====================

sovd_web_ui is a lightweight web application for browsing SOVD entity trees.
It connects to the ros2_medkit gateway and visualizes the entity hierarchy.

.. figure:: /_static/images/00_ui_view.png
   :alt: sovd_web_ui main interface
   :align: center
   :width: 600px

   The sovd_web_ui interface showing entity tree, detail panel, and data view.

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
   docker run -p 3000:80 ghcr.io/selfpatch/sovd_web_ui:latest

Then open http://localhost:3000 in your browser.

From Source
~~~~~~~~~~~

.. code-block:: bash

   # Clone the repository
   git clone https://github.com/selfpatch/sovd_web_ui.git
   cd sovd_web_ui

   # Install dependencies
   npm install

   # Build for production
   npm run build

   # Start development server (port 5173)
   npm run dev

Connecting to ros2_medkit
-------------------------

1. Open the web UI in your browser — you'll see the connection dialog:

   .. figure:: /_static/images/01a_connection_dialog.png
      :alt: Connection dialog
      :align: center
      :width: 600px

      Connection dialog prompting for gateway URL.

2. Enter the gateway URL (e.g., ``http://localhost:8080``) and base endpoint (e.g., ``api/v1``):

   .. figure:: /_static/images/01b_connection_dialog_filled.png
      :alt: Connection dialog with URL
      :align: center
      :width: 600px

      Enter your gateway URL and click Connect.

3. Browse the entity tree in the left sidebar:

   .. figure:: /_static/images/02_entity_tree_collapsed.png
      :alt: Entity tree collapsed
      :align: center
      :width: 600px

      Entity tree showing areas and components.

4. Expand nodes to see the hierarchy:

   .. figure:: /_static/images/03_entity_tree_expanded.png
      :alt: Entity tree expanded
      :align: center
      :width: 600px

      Expanded tree showing apps and virtual folders.

5. Click on any entity to view its details:

   .. figure:: /_static/images/04_app_entity_detail.png
      :alt: Entity detail panel
      :align: center
      :width: 600px

      Detail panel showing entity metadata and capabilities.

.. tip::

   If the gateway runs on a different host, ensure CORS is configured.
   See :doc:`/config/server` for CORS settings.

Using the Interface
-------------------

Data View
~~~~~~~~~

Click on the **data** folder under any entity to see available topics:

.. figure:: /_static/images/05_data_view.png
   :alt: Data view
   :align: center
   :width: 600px

   List of topics published by an entity.

Click on a specific topic to see its current value:

.. figure:: /_static/images/06_topic_data_view.png
   :alt: Topic data view
   :align: center
   :width: 600px

   Real-time topic data with JSON visualization.

Operations
~~~~~~~~~~

The **operations** folder shows available services and actions:

.. figure:: /_static/images/07_operations_panel.png
   :alt: Operations panel
   :align: center
   :width: 600px

   List of operations (services and actions) for an entity.

Execute an operation and see the result:

.. figure:: /_static/images/08_operations_execution.png
   :alt: Operation execution
   :align: center
   :width: 600px

   Operation execution with request/response display.

Configurations
~~~~~~~~~~~~~~

The **configurations** folder exposes ROS 2 node parameters:

.. figure:: /_static/images/09_configurations_list.png
   :alt: Configurations list
   :align: center
   :width: 600px

   List of parameters with current values.

Edit parameters directly in the UI:

.. figure:: /_static/images/10_configuration_edit.png
   :alt: Configuration edit
   :align: center
   :width: 600px

   Inline parameter editing with type validation.

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
