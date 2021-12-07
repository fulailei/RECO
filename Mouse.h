#pragma once
#include <QLabel>
#include <QPainter>
class Mouse : public QLabel
{
	Q_OBJECT
public:
	explicit Mouse(QWidget *parent = nullptr);


	//��갴��
	void mousePressEvent(QMouseEvent *ev);


	//����ͷ�
	void mouseReleaseEvent(QMouseEvent *ev);


	//����ƶ�
	virtual void  mouseMoveEvent(QMouseEvent *ev);

	void getXY(int cor[4]);

	void getcenter(QPaintEvent *ev);
	//��ͼ����
	void paintEvent(QPaintEvent *event);

signals:
private:
	bool m_isMousePress;
	QPoint m_beginPoint, m_endPoint;
	QPoint m_midPoint;
	QPainter m_painter;
};

