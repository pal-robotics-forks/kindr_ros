^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kindr_rviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2020-06-06)
------------------

0.3.1 (2020-03-16)
------------------
* Add SYSTEM to more include_directories
* Merge branch 'fix/package_xml_cleanup' into 'master'
  Cleanup of package.xml files
  GitOrigin-RevId: b5d8e56d14cb59338714cbc207c62af389d35826
* Merge branch 'fix/remove_or_update_weblinks_in_package_xml' into 'master'
  Fix/remove or update weblinks in package xml
  See merge request anybotics/anybotics!175
  GitOrigin-RevId: ba2d4fc878cf3f3870704102e19c02417a42d960
* Contributors: Gabriel Hottiger, Remo Diethelm, Victor Lopez

Forthcoming
-----------
* 0.3.2
* Updated Changelog
* 0.3.1
* Updated Changelog
* Add SYSTEM to more include_directories
* Merge branch 'feature/rd_cov_packages_cpp' into 'master'
  [cmake coverage] Coverage for some packages
  GitOrigin-RevId: 63a0e692a8ddd11e0f2f75638969ee4a465ab8b2
* Merge branch 'fix/update-cmake-minimum-version' into 'master'
  update cmake minimum version from 2.8.3 to 3.5.1
  GitOrigin-RevId: 3719820fca7758e5ce1feee3409a3cc6b0af17a3
* Merge pull request `#24 <https://github.com/pal-robotics-forks/kindr_ros/issues/24>`_ from devrite:fix/missing-ogre-colour-include
  GitOrigin-RevId: 8b77fd9db90549cf749d6429afdb89a4ad4d4b6e
* Add include missing ogre colour include
* Merge branch 'fix/package_xml_cleanup' into 'master'
  Cleanup of package.xml files
  GitOrigin-RevId: b5d8e56d14cb59338714cbc207c62af389d35826
* Merge branch 'fix/remove_or_update_weblinks_in_package_xml' into 'master'
  Fix/remove or update weblinks in package xml
  See merge request anybotics/anybotics!175
  GitOrigin-RevId: ba2d4fc878cf3f3870704102e19c02417a42d960
* Contributors: Enea Scioni, Gabriel Hottiger, Manuel Dätwiler, Markus Hofstaetter, Remo Diethelm, Sync Runner, Victor Lopez

0.3.0 (2019-03-14)
------------------
* another attempt to fix build error
* fix typo
* another attempt to fix the pthread issue
* add libpthread-stubs0-dev to fix the debian generation
* Merge pull request `#21 <https://github.com/pal-robotics-forks/kindr_ros/issues/21>`_ from ANYbotics/master
  add pthread for Ubuntu Bionic
* add pthread for Ubuntu Bionic
* Updated email address.
* Contributors: Remo Diethelm, Samuel Bachmann

0.2.3 (2018-11-29)
------------------

0.2.2 (2018-08-20)
------------------
* Merge pull request `#19 <https://github.com/pal-robotics-forks/kindr_ros/issues/19>`_ from ethz-asl/release
  0.2.1
* Contributors: remod

0.2.1 (2018-06-14)
------------------
* Merge branch 'release' of github.com:ethz-asl/kindr_ros
* Contributors: Remo Diethelm

0.2.0 (2018-03-22)
------------------
* Merge pull request `#17 <https://github.com/pal-robotics-forks/kindr_ros/issues/17>`_ from ethz-asl/release
  0.1.2
* Contributors: remod

0.1.2 (2017-11-24)
------------------
* Merge pull request `#15 <https://github.com/pal-robotics-forks/kindr_ros/issues/15>`_ from ethz-asl/fix/font_segfault
  let rviz movable select the font
* let rviz movable select the font
* Merge pull request `#13 <https://github.com/pal-robotics-forks/kindr_ros/issues/13>`_ from FGiraldez/fix/vector_at_position_font
  Changed font from Arial (removed in RViz 1.12.12) to Liberation Sans.
* Changed font from Arial (removed in RViz 1.12.12) to Liberation Sans.
* Merge pull request `#12 <https://github.com/pal-robotics-forks/kindr_ros/issues/12>`_ from ethz-asl/release
  0.1.1
* Contributors: Francisco Giraldez, Gabriel Hottiger, hogabrie, remod

0.1.1 (2017-08-08)
------------------
* Fixed build warnings.
* Merge pull request `#9 <https://github.com/pal-robotics-forks/kindr_ros/issues/9>`_ from ethz-asl/kinetic-devel
  make rviz plugin compatible with kinetic
* use rviz qt version
* Merge branch 'master' into kinetic-devel
* Merge branch 'master' into kinetic-devel
* [kindr_rviz_plugins] add rqt_gui_cpp dependency
* [kindr_rviz_plugins] qt5 ready
* Contributors: Remo Diethelm, Samuel Bachmann

0.1.0 (2017-01-30)
------------------
* 0.0.1
* Merge pull request `#8 <https://github.com/pal-robotics-forks/kindr_ros/issues/8>`_ from ethz-asl/feature/circle_arrow_torque
  Feature/circle arrow torque
* add comments to torque plugin
* Merge pull request `#7 <https://github.com/pal-robotics-forks/kindr_ros/issues/7>`_ from ethz-asl/fix/orientation
  Fix/orientation
* add circle arrow for type torque
* Fixed bug in position vector update.
* Add position variable.
* Fixed orientation of position offset.
* Merge pull request `#5 <https://github.com/pal-robotics-forks/kindr_ros/issues/5>`_ from gonond/master
  Fixing bug when msg-frame_id unequal global rviz-frame
* Inserted transform of position contents of messages into the global rviz-frame, before they are added to the position of the local frame's origin. This fixes a bug, namely that vector messages with nonzero position contents are placed incorrectly unless the global rviz-frame is set to the frame of the message header.
* Contributors: David Gonon, Gabriel, Péter Fankhauser, Samuel Bachmann, dbellicoso, hogabrie

0.0.1 (2015-09-18)
------------------
* Merge pull request `#3 <https://github.com/pal-robotics-forks/kindr_ros/issues/3>`_ from ethz-asl/fix/rviz-plugins
  fixed rviz plugins GUI update, works now with rqt_rviz
* fixed rviz plugins GUI update, works now with rqt_rviz
* Merge pull request `#2 <https://github.com/pal-robotics-forks/kindr_ros/issues/2>`_ from ethz-asl/feature/rviz_plugin_color
  Removed predefined color from vector at position
* removed commented code, cleanup, caption is now a bit lower
* Removed predefined color from vector at position
* Initial commit
* Contributors: Christian Gehring, Remo Diethelm, Samuel Bachmann, gehrinch, remod
