#include <QStringList>

#include "CamTreeItem.h"

#include "CamInfoTree.h"


CameraInfoTree::CameraInfoTree(const QStringList& iCameraNames, QObject* ipParent)
    : QAbstractItemModel(ipParent)
{
    QString root_data = "Camera Information:";
    mpRootItem = new CameraTreeItem(root_data);
    SetupModelData( iCameraNames, mpRootItem );
}

CameraInfoTree::~CameraInfoTree()
{
    delete mpRootItem;
}

int CameraInfoTree::columnCount(const QModelIndex& iParent) const
{
    if (iParent.isValid())
    {
        return static_cast<CameraTreeItem*>(iParent.internalPointer())->ColumnCount();
    }
    else
    {
        return mpRootItem->ColumnCount();
    }
}

QVariant CameraInfoTree::data(const QModelIndex& iIndex, int iRole) const
{
    if (!iIndex.isValid())
    {
        return QVariant();
    }

    if (Qt::DisplayRole != iRole )
    {
        return QVariant();
    }

    CameraTreeItem *item = static_cast<CameraTreeItem*>(iIndex.internalPointer());

    return item->Data();
}

Qt::ItemFlags CameraInfoTree::flags(const QModelIndex& iIndex) const
{
    if (!iIndex.isValid())
    {
        return 0;
    }

    return QAbstractItemModel::flags(iIndex);
}

QVariant CameraInfoTree::headerData(int iSection, Qt::Orientation iOrientation, int iRole) const
{
    if (iSection > 0)
    {
        //:KLUDGE: Here just so the function doesn't need to change its signaure.
    }
    if (iOrientation == Qt::Horizontal && iRole == Qt::DisplayRole)
    {
        return mpRootItem->Data();
    }

    return QVariant();
}

QModelIndex CameraInfoTree::index(int iRow, int iColumn, const QModelIndex& iParent)
            const
{
    if (!hasIndex(iRow, iColumn, iParent))
    {
        return QModelIndex();
    }

    CameraTreeItem *parentItem;

    if (!iParent.isValid())
    {
        parentItem = mpRootItem;
    }
    else
    {
        parentItem = static_cast<CameraTreeItem*>(iParent.internalPointer());
    }

    CameraTreeItem *childItem = parentItem->Child(iRow);
    if (childItem)
    {
        return createIndex(iRow, iColumn, childItem);
    }
    return QModelIndex();
}

QModelIndex CameraInfoTree::parent(const QModelIndex& iIndex) const
{
    if (!iIndex.isValid())
    {
        return QModelIndex();
    }

    CameraTreeItem *childItem = static_cast<CameraTreeItem*>(iIndex.internalPointer());
    CameraTreeItem *parentItem = childItem->ParentItem();

    if (parentItem == mpRootItem)
    {
        return QModelIndex();
    }

    return createIndex(parentItem->Row(), 0, parentItem);
}

int CameraInfoTree::rowCount(const QModelIndex& iParent) const
{
    CameraTreeItem *parentItem;
    if (iParent.column() > 0)
    {
        return 0;
    }

    if (!iParent.isValid())
    {
        parentItem = mpRootItem;
    }
    else
    {
        parentItem = static_cast<CameraTreeItem*>(iParent.internalPointer());
    }

    return parentItem->ChildCount();
}

void CameraInfoTree::SetupModelData(const QStringList& iCameraNames, CameraTreeItem* ipParent)
{
    QList<CameraTreeItem*> parents;
    QList<int> indentations;
    parents << ipParent;
    ipParent->AppendChild( new CameraTreeItem( QString("Cameras Found"), ipParent ) );
    ipParent->AppendChild( new CameraTreeItem( QString("Camera Config File"), ipParent ));

    mpCameraInfoPtr = parents.last()->Child( parents.last()->ChildCount()-2 );
    parents << mpCameraInfoPtr;

    if (iCameraNames.size() > 0)
    {
        foreach (QString name, iCameraNames)
        {
            mpCameraInfoPtr->AppendChild( new CameraTreeItem( name, mpCameraInfoPtr ) );
        }
    }
    else
    {
        mpCameraInfoPtr->AppendChild( new CameraTreeItem( QString("No Camera Devices Found."), mpCameraInfoPtr ) );
    }

    mpConfigInfoPtr = ipParent->Child( ipParent->ChildCount()-1 );
    parents << mpConfigInfoPtr;
    mpConfigInfoPtr->AppendChild( new CameraTreeItem( QString("None Loaded."), mpConfigInfoPtr ));
}

void CameraInfoTree::UpdateConfigFileInfo( const QString& iFilename )
{
    mpConfigInfoPtr->Child( mpConfigInfoPtr->ChildCount()-1 )->UpdateData( iFilename );
}
