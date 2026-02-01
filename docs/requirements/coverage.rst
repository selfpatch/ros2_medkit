Coverage Report
===============

This page provides an overview of requirement coverage by tests.

.. note::

   Coverage is calculated based on the ``verifies`` link between test cases
   and requirements. A requirement is considered **verified** when at least
   one test case links to it using ``:verifies: REQ_xxx``.

Overall Coverage
----------------

.. grid:: 2
   :gutter: 3

   .. grid-item-card:: Verification Status
      :class-card: sd-border-0

      .. needpie:: Requirements Coverage
         :labels: Verified, Not Verified
         :colors: #2e7d32, #c62828
         :text_color: #ffffff

         type == 'req' and len(verifies_back) > 0
         type == 'req' and len(verifies_back) == 0

   .. grid-item-card:: Summary
      :class-card: sd-border-0

      **Key Metrics:**

      - Total requirements tracked from SOVD standard
      - Test cases link to requirements via ``:verifies:`` option
      - See :doc:`verification` for full test case documentation

Coverage by Category
--------------------

.. tab-set::

   .. tab-item:: Discovery

      .. needtable::
         :filter: type == 'req' and "Discovery" in tags
         :columns: id, title, status, verifies_back
         :style: table

   .. tab-item:: Data

      .. needtable::
         :filter: type == 'req' and "Data" in tags
         :columns: id, title, status, verifies_back
         :style: table

   .. tab-item:: Operations

      .. needtable::
         :filter: type == 'req' and "Operations" in tags
         :columns: id, title, status, verifies_back
         :style: table

   .. tab-item:: Configuration

      .. needtable::
         :filter: type == 'req' and "Configuration" in tags
         :columns: id, title, status, verifies_back
         :style: table

   .. tab-item:: Faults

      .. needtable::
         :filter: type == 'req' and "Faults" in tags
         :columns: id, title, status, verifies_back
         :style: table

   .. tab-item:: Auth

      .. needtable::
         :filter: type == 'req' and "Auth" in tags
         :columns: id, title, status, verifies_back
         :style: table

Missing Verification
--------------------

The following requirements do not have any linked test cases yet:

.. needtable::
   :filter: type == 'req' and len(verifies_back) == 0
   :columns: id, title, tags
   :style: table
   :sort: id

Full Traceability Matrix
------------------------

Complete mapping of requirements to their verifying test cases:

.. needtable::
   :filter: type == 'req'
   :columns: id, title, tags, verifies_back
   :style: table
   :sort: id
