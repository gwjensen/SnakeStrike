#ifndef CameraTreeItem_h
#define CameraTreeItem_h

#include <QList>
#include <QVariant>

class CameraTreeItem
{
public:
    explicit CameraTreeItem( const QString& iData, CameraTreeItem* ipParentItem = 0 );
    ~CameraTreeItem();

    void AppendChild( CameraTreeItem* iChild );

    CameraTreeItem* Child( int iRow );
    int ChildCount() const;
    int ColumnCount() const;
    QVariant Data() const;
    void UpdateData( QVariant iNewData );
    int Row() const;
    CameraTreeItem* ParentItem();


private:
    QList<CameraTreeItem*> mChildItems;
    CameraTreeItem* mpParentItem;
    QVariant mItemData;

};

#endif // CameraTreeItem_h
