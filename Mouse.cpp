#include "Mouse.h"
#include<QDebug>
#include <QMouseEvent>
#include <QPainter>
#include <QRect>
#include<QLabel>
Mouse::Mouse(QWidget *parent) : QLabel(parent)
{

}
//鼠标按下
void Mouse::mousePressEvent(QMouseEvent *ev)
{
	//当鼠标左键按下  提示信息
	if (ev->button() == Qt::LeftButton)
	{
		m_isMousePress = true;




		//获取点坐标
		m_beginPoint = ev->pos();
		qDebug() << "00" << m_beginPoint;
		//update();
	}
}


//鼠标释放
void Mouse::mouseReleaseEvent(QMouseEvent *ev)
{
	if (ev->button() == Qt::LeftButton)
	{
		m_endPoint = ev->pos();
		m_isMousePress = false;
		qDebug() << "00" << m_endPoint;
		update();
	}


}
//鼠标移动，更新矩形框
void Mouse::mouseMoveEvent(QMouseEvent *ev)
{
	if (ev->buttons() &   Qt::LeftButton)
	{
		m_midPoint = ev->pos();
		update();
	}
}


//画矩形框


void Mouse::paintEvent(QPaintEvent *ev)
{
	QLabel::paintEvent(ev);//先调用父类的paintEvent为了显示'背景'!!!
//    qDebug()<<"开始画";
//    qDebug()<<m_beginPoint;
//    qDebug()<<m_endPoint;
	QPainter m_painter(this);


	m_painter.setPen(QPen(Qt::red, 2));




	//    //设置画刷
	//    QBrush brush(Qt::red);
	//    //设置画刷风格
	//    brush.setStyle(Qt::Dense7Pattern);
	//    //让画家使用画刷
	//    m_painter.setBrush(brush);
		//m_painter.drawRect(QRect(m_beginPoint,m_endPoint));
	if (m_isMousePress)
	{
		m_painter.drawRect(QRect(m_beginPoint, m_midPoint));
	}
	else
	{
		m_painter.drawRect(QRect(m_beginPoint, m_endPoint));
	}

}
void Mouse::getcenter(QPaintEvent *ev)
{
	QLabel::paintEvent(ev);//先调用父类的paintEvent为了显示'背景'!!!
//    qDebug()<<"开始画";
//    qDebug()<<m_beginPoint;
//    qDebug()<<m_endPoint;
	QPainter m_painter(this);
	m_painter.setPen(QPen(Qt::green, 2));
	m_painter.setRenderHint(QPainter::Antialiasing, true);  //设置渲染,启动反锯齿
	m_painter.drawEllipse(40.0, 40.0, 100.0, 100.0);   //半径为50的圆
}
void Mouse::getXY(int cor[4])
{
	cor[0] = m_beginPoint.x();
	cor[1] = m_beginPoint.y();
	cor[2] = m_endPoint.x();
	cor[3] = m_endPoint.y();
}
