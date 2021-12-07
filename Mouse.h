#pragma once
#include <QLabel>
#include <QPainter>
class Mouse : public QLabel
{
	Q_OBJECT
public:
	explicit Mouse(QWidget *parent = nullptr);


	//鼠标按下
	void mousePressEvent(QMouseEvent *ev);


	//鼠标释放
	void mouseReleaseEvent(QMouseEvent *ev);


	//鼠标移动
	virtual void  mouseMoveEvent(QMouseEvent *ev);

	void getXY(int cor[4]);

	void getcenter(QPaintEvent *ev);
	//绘图操作
	void paintEvent(QPaintEvent *event);

signals:
private:
	bool m_isMousePress;
	QPoint m_beginPoint, m_endPoint;
	QPoint m_midPoint;
	QPainter m_painter;
};

