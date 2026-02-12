^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package maliput_sparse
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.5 (2026-02-12)
------------------
* Fix BCR and add bazel workflow. (`#72 <https://github.com/maliput/maliput_sparse/issues/72>`_)
* Some fixes for macos build (`#71 <https://github.com/maliput/maliput_sparse/issues/71>`_)
* Contributors: Franco Cipollone

0.2.4 (2026-02-09)
------------------
* Setup package for BCR. (`#68 <https://github.com/maliput/maliput_sparse/issues/68>`_)
* Bazel support (`#67 <https://github.com/maliput/maliput_sparse/issues/67>`_)
* Implement Lane::DoGetCurvature. (`#63 <https://github.com/maliput/maliput_sparse/issues/63>`_)
* `RangeValidator` default template. (`#62 <https://github.com/maliput/maliput_sparse/issues/62>`_)
* Update maliput common error lib. (`#61 <https://github.com/maliput/maliput_sparse/issues/61>`_)
* Fixes CI. (`#60 <https://github.com/maliput/maliput_sparse/issues/60>`_)
* Update github action versions. (`#59 <https://github.com/maliput/maliput_sparse/issues/59>`_)
* Updates ros-tooling verison to avoid error with dependency. (`#58 <https://github.com/maliput/maliput_sparse/issues/58>`_)
* Uses compare methods instead of maliput test utilities. (`#57 <https://github.com/maliput/maliput_sparse/issues/57>`_)
* Some fixes for clang build and sanitizers checks (`#56 <https://github.com/maliput/maliput_sparse/issues/56>`_)
* Matches with change in logger format. (`#55 <https://github.com/maliput/maliput_sparse/issues/55>`_)
* Contributors: Agustin Alba Chicar, Franco Cipollone, Santiago Lopez

0.2.3 (2023-02-15)
------------------
* Relies on GetClosestPoint for the GetClosestPointUsing2dProjection method. (`#53 <https://github.com/maliput/maliput_sparse/issues/53>`_)
* Stores linestring's points in a kdtree. (`#52 <https://github.com/maliput/maliput_sparse/issues/52>`_)
* Precomputes linestring's segments intervals (`#51 <https://github.com/maliput/maliput_sparse/issues/51>`_)
* Contributors: Franco Cipollone

0.2.2 (2023-01-06)
------------------
* Fixes bug due to segments with zero length. (`#49 <https://github.com/maliput/maliput_sparse/issues/49>`_)
* Contributors: Franco Cipollone

0.2.1 (2023-01-04)
------------------
* Implements range validator for the geometry::utils methods. (`#48 <https://github.com/maliput/maliput_sparse/issues/48>`_)
* Uses the closest point to calculate the equivalent p in a boundary. (`#46 <https://github.com/maliput/maliput_sparse/issues/46>`_)
* Adds a adjacency checker for the builder. (`#45 <https://github.com/maliput/maliput_sparse/issues/45>`_)
* Contributors: Franco Cipollone

0.2.0 (2022-12-12)
------------------
* Builds up RoadGeometry out of parsed information. (`#40 <https://github.com/maliput/maliput_sparse/issues/40>`_)
* Contributors: Franco Cipollone

0.1.0 (2022-11-28)
------------------
* Improves documentation. (`#43 <https://github.com/maliput/maliput_sparse/issues/43>`_)
* Uses RangeValidator within LaneGeometry. (`#42 <https://github.com/maliput/maliput_sparse/issues/42>`_)
* Supports superelevation. (`#38 <https://github.com/maliput/maliput_sparse/issues/38>`_)
* Supports passing custom centerline to the lane. (`#37 <https://github.com/maliput/maliput_sparse/issues/37>`_)
* Fixes minor warning. (`#36 <https://github.com/maliput/maliput_sparse/issues/36>`_)
* Update triage.yml (`#35 <https://github.com/maliput/maliput_sparse/issues/35>`_)
* Overrides pendings RoadGeometry methods. (`#29 <https://github.com/maliput/maliput_sparse/issues/29>`_)
* Builder for BranchPoints (`#24 <https://github.com/maliput/maliput_sparse/issues/24>`_)
* Adds compare method for the LineStrings. (`#28 <https://github.com/maliput/maliput_sparse/issues/28>`_)
* Lane::DoEvalMotionDerivatives. (`#27 <https://github.com/maliput/maliput_sparse/issues/27>`_)
* Implements do_segment_bounds and DoToSegmentPositionBackend. (`#23 <https://github.com/maliput/maliput_sparse/issues/23>`_)
* Builder API for the RoadGeometry (`#11 <https://github.com/maliput/maliput_sparse/issues/11>`_)
* Implements Lane::DoToLanePositionBackend. (`#22 <https://github.com/maliput/maliput_sparse/issues/22>`_)
* Adds r-bounds computations. (`#19 <https://github.com/maliput/maliput_sparse/issues/19>`_)
* Adds a base class for RoadGeometry. (`#21 <https://github.com/maliput/maliput_sparse/issues/21>`_)
* Expose lane_geometry header file as it is required by the builder API. (`#20 <https://github.com/maliput/maliput_sparse/issues/20>`_)
* Adds Lane and implements LaneGeometry::WDot and LaneGeometry::WInverse. (`#13 <https://github.com/maliput/maliput_sparse/issues/13>`_)
* Adds LaneGeometry class. (`#8 <https://github.com/maliput/maliput_sparse/issues/8>`_)
* Rename maliput_sample folder to maliput_sparse. (`#12 <https://github.com/maliput/maliput_sparse/issues/12>`_)
* Adds method for computing the centerline. (`#9 <https://github.com/maliput/maliput_sparse/issues/9>`_)
* Adds LineString (`#4 <https://github.com/maliput/maliput_sparse/issues/4>`_)
* Initial project skeleton. (`#2 <https://github.com/maliput/maliput_sparse/issues/2>`_)
* Contributors: Agustin Alba Chicar, Franco Cipollone
