#ifndef CameraInfoTree_h
#define CameraInfoTree_h

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>

class CameraTreeItem;

class CameraInfoTree : public QAbstractItemModel
{
    Q_OBJECT

    public:
        explicit CameraInfoTree(const QStringList& iData, QObject* ipParent = 0);
        ~CameraInfoTree();

        QVariant data(const QModelIndex& iIndex, int iRole) const override;
        Qt::ItemFlags flags(const QModelIndex& iIndex) const override;
        QVariant headerData(int iSection, Qt::Orientation iOrientation,
                            int iRole = Qt::DisplayRole) const override;
        QModelIndex index(int iRow, int iColumn,
                          const QModelIndex& iParent = QModelIndex()) const override;
        QModelIndex parent(const QModelIndex& iIndex) const override;
        int rowCount(const QModelIndex& iParent = QModelIndex()) const override;
        int columnCount(const QModelIndex& iParent = QModelIndex()) const override;
        void UpdateConfigFileInfo( const QString& iFilename );

    private:
        void SetupModelData(const QStringList& iCameraNames, CameraTreeItem* iParent);

        CameraTreeItem* mpRootItem;

        CameraTreeItem* mpCameraInfoPtr;
        CameraTreeItem* mpConfigInfoPtr;
};
#endif // CameraInfoTree_h
