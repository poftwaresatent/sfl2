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

#include "QtNepumuk.hpp"


int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  NepumukWindow window(&app);
  window.show();
  return app.exec();
}


//////////////////////////////////////////////////


NepumukWidget::
NepumukWidget(QWidget * parent)
	: QGLWidget(parent)
{
}

QSize NepumukWidget::
minimumSizeHint() const
{
  return QSize(150, 150);
}


QSize NepumukWidget::
sizeHint() const
{
  return QSize(700, 500);
}


void NepumukWidget::
setFoo(int foo)
{
	if(foo == m_foo)
		return;
	m_foo = foo;
	emit fooChanged(foo);
}


void NepumukWidget::
initializeGL()
{
  glClearColor(0, 0, 0, 0);
}


void NepumukWidget::
paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT);
	////  m_simulator->Draw();
}


void NepumukWidget::
resizeGL(int width, int height)
{
	////  simulator->Reshape(width, height);
}


//////////////////////////////////////////////////


NepumukWindow::
NepumukWindow(QCoreApplication * app)
{
	QHBoxLayout * hbox(new QHBoxLayout(this));
	QVBoxLayout * vbox(new QVBoxLayout(this));
	hbox->addLayout(vbox);
	
	NepumukWidget * npm(new NepumukWidget(this));
	hbox->addWidget(npm);
	
	QGroupBox * world_group(new QGroupBox("World", this));
	QRadioButton * world_boiler(new QRadioButton("use boilerplate"));
	QRadioButton * world_file(new QRadioButton("use config file"));
	QRadioButton * world_travmap(new QRadioButton("use travmap file"));
	world_boiler->setChecked(true);
	QVBoxLayout * world_vbox(new QVBoxLayout(this));
	world_vbox->addWidget(world_boiler);
	world_vbox->addWidget(world_file);
	world_vbox->addWidget(world_travmap);
	world_vbox->addStretch(1);
	world_group->setLayout(world_vbox);
	vbox->addWidget(world_group);
	
	QPushButton * b_quit(new QPushButton("QUIT", this));
	vbox->addWidget(b_quit);
	connect(b_quit, SIGNAL(clicked()), app, SLOT(quit()));
	
	setLayout(hbox);
	setWindowTitle("Nepumuk");
}
