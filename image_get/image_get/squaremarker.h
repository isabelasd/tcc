#ifndef SQUAREMARKER_H
#define SQUAREMARKER_H

#include <QGraphicsItem>
#include <QPainter>
#include <QGraphicsRectItem>

class squaremarker : public QGraphicsItem
{
public:
    squaremarker();

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    qreal x_m , y_m, w_m, h_m ;

protected :
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
};

#endif // SQUAREMARKER_H
