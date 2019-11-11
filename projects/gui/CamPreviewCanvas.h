#ifndef CamPreviewCanvas_h
#define CamPreviewCanvas_h

#include <QLabel>
#include <QPixmap>
#include <QResizeEvent>

class CamPreviewCanvas : public QLabel
{
    Q_OBJECT
    public:
        explicit CamPreviewCanvas(QWidget* ipParent = 0);
        ~CamPreviewCanvas();
        virtual int heightForWidth( int iWidth ) const;
        virtual QSize sizeHint() const;
        QPixmap ScaledPixmap() const;

    public slots:
        void setPixmap ( const QPixmap& iPixmap);
        void resizeEvent(QResizeEvent* ipEvent);

    private:
        QPixmap mPixmap;
};

#endif // CamPreviewCanvas_h
