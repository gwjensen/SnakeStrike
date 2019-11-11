---
fields:
  mpRecordDialog:
    type: RecordDialog *
    annotation:
      - private
    description: __MISSING__
  mDir:
    type: QDir
    annotation:
      - private
    description: __MISSING__
  mCamWindowList:
    type: QVector<CamPreviewWindow *>
    annotation:
      - private
    description: __MISSING__
  mpFileDialog:
    annotation:
      - private
    description: __MISSING__
    type: QFileDialog *
  mpCamList:
    type: GuiCamArray *
    annotation:
      - private
    description: __MISSING__
  mpCamCalDialog:
    annotation:
      - private
    type: CamCalibrateDialog *
    description: __MISSING__
  mpExportDialog:
    type: ExportDialog *
    description: __MISSING__
    annotation:
      - private
  mCamPreviewList:
    description: __MISSING__
    annotation:
      - private
    type: QVector<CamPreviewCanvas *>
  mpVis3dDialog:
    annotation:
      - private
    type: VisualizationDialog *
    description: __MISSING__
  mpTrackDialog:
    description: __MISSING__
    annotation:
      - private
    type: TrackingDialog *
  mpProjDialog:
    type: ProjectDialog *
    annotation:
      - private
    description: __MISSING__
  mPlay:
    description: __MISSING__
    type: bool
    annotation:
      - private
  mCamHerz:
    annotation:
      - private
    description: __MISSING__
    type: uint32_t
  staticMetaObject:
    type: const QMetaObject
    description: __MISSING__
  mpTreeModel:
    annotation:
      - private
    type: CameraInfoTree *
    description: __MISSING__
  mpUi:
    annotation:
      - private
    description: __MISSING__
    type: Ui::MainWindow *
  mpHelpDialog:
    description: __MISSING__
    type: HelpDialog *
    annotation:
      - private
owner: gwjensen
brief: The main window of the Gui.
declaration: "\nclass MainWindow;"
tags:
  - class
defined-in-file: "projects/gui/MainWindow.h"
layout: class
title: MainWindow
---
