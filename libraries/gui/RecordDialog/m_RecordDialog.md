---
defined-in-file: "projects/gui/RecordDialog.h"
title: RecordDialog
tags:
  - method
overloads:
  explicit RecordDialog(GuiCamArray *&, const ProjectDialog *&, const RecordType &, uint32_t, bool, QWidget *):
    arguments:
      - name: iCamList
        type: GuiCamArray *&
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: const ProjectDialog *&
        name: iProjDetails
      - type: const RecordType &
        description: __OPTIONAL__
        name: iRecordAction
      - name: iCamHerz
        type: uint32_t
        description: __OPTIONAL__
      - description: __OPTIONAL__
        type: bool
        name: iXmlAlreadyExists
      - type: QWidget *
        name: ipParent
        description: __OPTIONAL__
    description: __MISSING__
    return: __OPTIONAL__
    signature_with_names: explicit RecordDialog(GuiCamArray *& iCamList, const ProjectDialog *& iProjDetails, const RecordType & iRecordAction, uint32_t iCamHerz, bool iXmlAlreadyExists, QWidget * ipParent)
  RecordDialog(const RecordDialog &):
    description: __MISSING__
    signature_with_names: RecordDialog(const RecordDialog &)
    arguments:
      - name: unnamed-0
        unnamed: true
        description: __OPTIONAL__
        type: const RecordDialog &
    return: __OPTIONAL__
brief:
is_ctor: true
layout: method
owner: gwjensen
---
