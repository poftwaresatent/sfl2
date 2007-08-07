/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007 Roland Philippsen <roland dot philippsen at gmx dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

#ifndef NPM_QT_NEPUMUK_HPP
#define NPM_QT_NEPUMUK_HPP


#include <QApplication>
#include <QWidget>
#include <QtGui>
#include <QGLWidget>
#include <QtOpenGL>
#include <boost/shared_ptr.hpp>


namespace npm {
	class World;
	class Simulator;
}


class NepumukWidget
  : public QGLWidget
{
  Q_OBJECT  

public:
  NepumukWidget(QWidget * parent);
  
	bool InitWorldFromBoilerplate(const QString & name);
	bool InitWorldFromFile(const QString & filename);
	bool InitWorldFromTraversability(const QString & filename);
	bool HaveWorld() const;
	
	/** \pre HaveWorld() */
	bool InitSimulator(const QString & robot_config_filename,
										 const QString & layout_config_filename);
	
	
  QSize minimumSizeHint() const;
  QSize sizeHint() const;
  
public slots:
  void setFoo(int foo);
  
signals:
  void fooChanged(int foo);
  
protected:
  void initializeGL();
  void paintGL();
  void resizeGL(int width, int height);
	
private:
	int m_foo;
	
  boost::shared_ptr<npm::World> m_world;
	boost::shared_ptr<npm::Simulator> m_simulator;
};


class WorldGroup
	: public QGroupBox
{
  Q_OBJECT

public:
	WorldGroup(QWidget * parent);

public slots:
  void rBoilerToggled(bool on);
	void rFileToggled(bool on);
	void rTravmapToggled(bool on);
	void bConfigClicked();	
	void dConfigAccepted();	
	void bTravmapClicked();	
	void dTravmapAccepted();	
	void bCreateClicked();
	
private:
	QGridLayout * grid;
	QRadioButton * r_boiler, * r_config, * r_travmap;
	QLineEdit * e_config, * e_travmap;
	QPushButton * b_config, * b_travmap, * b_create;
	QFileDialog * d_config, * d_travmap;
};


class NepumukWindow
  : public QWidget
{
  Q_OBJECT

public:
  NepumukWindow(QCoreApplication * app);

private:
	NepumukWidget * npm;
	WorldGroup * world;
	QPushButton * b_quit;
};

#endif // NPM_QT_NEPUMUK_HPP
