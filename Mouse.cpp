#include "Mouse.h"
#include<QDebug>
#include <QMouseEvent>
#include <QPainter>
#include <QRect>
#include<QLabel>
Mouse::Mouse(QWidget *parent) : QLabel(parent)
{

}
//��갴��
void Mouse::mousePressEvent(QMouseEvent *ev)
{
	//������������  ��ʾ��Ϣ
	if (ev->button() == Qt::LeftButton)
	{
		m_isMousePress = true;




		//��ȡ������
		m_beginPoint = ev->pos();
		qDebug() << "00" << m_beginPoint;
		//update();
	}
}


//����ͷ�
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
//����ƶ������¾��ο�
void Mouse::mouseMoveEvent(QMouseEvent *ev)
{
	if (ev->buttons() &   Qt::LeftButton)
	{
		m_midPoint = ev->pos();
		update();
	}
}


//�����ο�


void Mouse::paintEvent(QPaintEvent *ev)
{
	QLabel::paintEvent(ev);//�ȵ��ø����paintEventΪ����ʾ'����'!!!
//    qDebug()<<"��ʼ��";
//    qDebug()<<m_beginPoint;
//    qDebug()<<m_endPoint;
	QPainter m_painter(this);


	m_painter.setPen(QPen(Qt::red, 2));




	//    //���û�ˢ
	//    QBrush brush(Qt::red);
	//    //���û�ˢ���
	//    brush.setStyle(Qt::Dense7Pattern);
	//    //�û���ʹ�û�ˢ
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
	QLabel::paintEvent(ev);//�ȵ��ø����paintEventΪ����ʾ'����'!!!
//    qDebug()<<"��ʼ��";
//    qDebug()<<m_beginPoint;
//    qDebug()<<m_endPoint;
	QPainter m_painter(this);
	m_painter.setPen(QPen(Qt::green, 2));
	m_painter.setRenderHint(QPainter::Antialiasing, true);  //������Ⱦ,���������
	m_painter.drawEllipse(40.0, 40.0, 100.0, 100.0);   //�뾶Ϊ50��Բ
}
void Mouse::getXY(int cor[4])
{
	cor[0] = m_beginPoint.x();
	cor[1] = m_beginPoint.y();
	cor[2] = m_endPoint.x();
	cor[3] = m_endPoint.y();
}
