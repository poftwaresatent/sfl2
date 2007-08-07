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
#include <stdlib.h>


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


WorldGroup::
WorldGroup(QWidget * parent)
	: QGroupBox("World", parent)
{
	grid = new QGridLayout(this);
	
	r_boiler = new QRadioButton("Boilerplate", this);
	grid->addWidget(r_boiler, 0, 0);
	r_boiler->setChecked(true);
	connect(r_boiler, SIGNAL(toggled(bool)), this, SLOT(rBoilerToggled(bool)));
	
	r_config = new QRadioButton("Config File", this);
	grid->addWidget(r_config, 1, 0);
	connect(r_config, SIGNAL(toggled(bool)), this, SLOT(rFileToggled(bool)));
	e_config = new QLineEdit(this);
	grid->addWidget(e_config, 1, 1);
	e_config->setReadOnly(true);
	e_config->setMinimumWidth(150);
	b_config = new QPushButton("Select", this);
	grid->addWidget(b_config, 1, 2);
	b_config->setDisabled(true);
	connect(b_config, SIGNAL(clicked()), this, SLOT(bConfigClicked()));
	d_config = 0;
	
	r_travmap = new QRadioButton("Travmap File");
	grid->addWidget(r_travmap, 2, 0);
	connect(r_travmap, SIGNAL(toggled(bool)), this, SLOT(rTravmapToggled(bool)));
	e_travmap = new QLineEdit(this);
	grid->addWidget(e_travmap, 2, 1);
	e_travmap->setReadOnly(true);		
	e_travmap->setMinimumWidth(150);
	b_travmap = new QPushButton("Select", this);
	grid->addWidget(b_travmap, 2, 2);
	b_travmap->setDisabled(true);
	connect(b_travmap, SIGNAL(clicked()), this, SLOT(bTravmapClicked()));
	d_travmap = 0;
	
	b_create = new QPushButton("Create", this);
	grid->addWidget(b_create, 3, 0, 1, 3);
	connect(b_create, SIGNAL(clicked()), this, SLOT(bCreateClicked()));
	
	setLayout(grid);
}


void WorldGroup::
rBoilerToggled(bool on)
{
}


void WorldGroup::
rFileToggled(bool on)
{
	b_config->setDisabled( ! on);
}


void WorldGroup::
rTravmapToggled(bool on)
{
	b_travmap->setDisabled( ! on);
}


void WorldGroup::
bConfigClicked()
{
	if(0 == d_config){
		d_config = new QFileDialog(this, "world config", getenv("PWD"));
		connect(d_config, SIGNAL(accepted()), this, SLOT(dConfigAccepted()));
		d_config->setFileMode(QFileDialog::ExistingFile);
		d_config->setAcceptMode(QFileDialog::AcceptOpen);
	}
	d_config->setVisible(true);
}


void WorldGroup::
dConfigAccepted()
{
	QStringList qsl(d_config->selectedFiles());
	if( ! qsl.empty())
		e_config->setText(qsl.first());
}


void WorldGroup::
bTravmapClicked()
{
	if(0 == d_travmap){
		d_travmap = new QFileDialog(this, "world travmap", getenv("PWD"));
		connect(d_travmap, SIGNAL(accepted()), this, SLOT(dTravmapAccepted()));
		d_travmap->setFileMode(QFileDialog::ExistingFile);
		d_travmap->setAcceptMode(QFileDialog::AcceptOpen);
	}
	d_travmap->setVisible(true);
}


void WorldGroup::
dTravmapAccepted()
{
	QStringList qsl(d_travmap->selectedFiles());
	if( ! qsl.empty())
		e_travmap->setText(qsl.first());
}


void WorldGroup::
bCreateClicked()
{
}


NepumukWindow::
NepumukWindow(QCoreApplication * app)
{
	QHBoxLayout * hbox(new QHBoxLayout(this));
	QVBoxLayout * vbox(new QVBoxLayout(this));
	hbox->addLayout(vbox);
	
	npm = new NepumukWidget(this);
	hbox->addWidget(npm);
	
	world = new WorldGroup(this);
	vbox->addWidget(world);
	vbox->addStretch(1);
	
	b_quit = new QPushButton("QUIT", this);
	vbox->addWidget(b_quit);
	connect(b_quit, SIGNAL(clicked()), app, SLOT(quit()));
	// should also quit when "last" (this) window closes
	
	setLayout(hbox);
	setWindowTitle("Nepumuk");
}
