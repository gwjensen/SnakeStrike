#include <QStringList>

#include "CamTreeItem.h"

CameraTreeItem::CameraTreeItem( const QString& iData, CameraTreeItem* ipParentItem )
    : mpParentItem( ipParentItem ),
      mItemData( iData )
{ }

CameraTreeItem::~CameraTreeItem()
{
    qDeleteAll( mChildItems );
}

void CameraTreeItem::AppendChild( CameraTreeItem* iItem )
{
    mChildItems.append( iItem );
}

CameraTreeItem* CameraTreeItem::Child( int iRow )
{
    return mChildItems.value( iRow );
}

int CameraTreeItem::ChildCount() const
{
    return mChildItems.count();
}

int CameraTreeItem::ColumnCount() const
{
    return 1;//m_itemData.count();
}

QVariant CameraTreeItem::Data() const
{
    return mItemData;
}

void CameraTreeItem::UpdateData( QVariant iNewData )
{
    mItemData = iNewData;
}

CameraTreeItem *CameraTreeItem::ParentItem()
{
    return mpParentItem;
}

int CameraTreeItem::Row() const
{
    if (mpParentItem)
    {
        return mpParentItem->mChildItems.indexOf(const_cast<CameraTreeItem*>(this));
    }

    return 0;
}
