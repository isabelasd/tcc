#include "squaremarker.h"

squaremarker::squaremarker()
{
    setFlag(QGraphicsItem::ItemIsMovable);
    setAcceptHoverEvents(true);
}

QRectF squaremarker::boundingRect() const
{
    return QRectF (0,0,w_m,h_m);
}

void squaremarker::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QRectF rect = boundingRect();
    QPen blackpen(Qt::black);
    painter->setPen(blackpen);
    painter->drawRect(rect);
}

void squaremarker::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mouseMoveEvent(event);
    if (x() < 0)
           setPos(0, y());
       else if (x() > 320 - w_m)
           setPos(320 - w_m, y());

       if (y() < 0)
           setPos(x(), 0);
       else if (y() > 240 - h_m)
           setPos(x(), 240 - h_m);

}
