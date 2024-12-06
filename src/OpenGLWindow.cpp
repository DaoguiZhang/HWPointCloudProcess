#include "OpenGLWindow.h"
#define M_PI       3.14159265358979323846

OpenGLWindow::OpenGLWindow(QWidget * parent)
	:QGLWidget(parent) 
{
	w = 1080, h = 720;
	fov_ = 60;
	globalArcBall_ = new ArcBallT(this->size().width(), this->size().height());
	scale_ = 1;
	mvMatrix_origin_.lookAt(QVector3D(19.9909f, -23.2952f, 5.94473f), QVector3D(19.9909f - 0.267769f, -23.2952f + 0.894322f, 5.94473f - 0.358452f), QVector3D(-0.18173f, 0.318479f, 0.930347f));
}

OpenGLWindow::~OpenGLWindow()
{
	for (std::vector<QOpenGLBuffer*>::iterator it = VBOs_.begin(); it != VBOs_.end(); it++) {
		if (*it != NULL) {
			delete *it;
			*it = NULL;
		}
	}
	for (std::vector<QOpenGLVertexArrayObject*>::iterator it = VAOs_.begin(); it != VAOs_.end(); it++) {
		if (*it != NULL) {
			delete *it;
			*it = NULL;
		}
	}
	for (std::vector<QOpenGLTexture*>::iterator it = textures_.begin(); it != textures_.end(); it++) {
		if (*it != NULL) {
			delete *it;
			*it = NULL;
		}
	}
	if (globalArcBall_ != NULL) delete globalArcBall_;
	if (model_ != NULL) delete model_;

}


void OpenGLWindow::loadCameraSlot()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("import meshlab camera"));
	QFile file(fileName);
	if (!file.open(QIODevice::ReadOnly)) {
		std::cout << "Can't open the file!" << std::endl;
	}
	QDomDocument doc;
	if (!doc.setContent(&file))
	{
		file.close();
		std::cout << "Can't set to doc!" << std::endl;
		return;
	}

	file.close();

	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> rotation_matrix, trans_matrix;
	QDomElement root = doc.documentElement();
	QDomNode node = root.firstChild();
	while (!node.isNull())
	{
		if (QString::compare(node.nodeName(), "VCGCamera") == 0) {
			QDomNamedNodeMap attr = node.attributes();
			int2 ViewportPx;
			ViewportPx.x= attr.namedItem("ViewportPx").nodeValue().section(' ', 0, 0).toInt();
			ViewportPx.y = attr.namedItem("ViewportPx").nodeValue().section(' ', 1, 1).toInt();
			
			for (int i = 0; i < 16; i++) {
				rotation_matrix(i) = attr.namedItem("RotationMatrix").nodeValue().section(' ', i, i).toFloat();
			}
			trans_matrix.setIdentity();
			for (int i = 0; i < 3; i++) {
				trans_matrix(i, 3) = attr.namedItem("TranslationVector").nodeValue().section(' ', i, i).toFloat();
			}
			float FocalMm= attr.namedItem("FocalMm").nodeValue().section(' ', 0, 0).toFloat();
			float2 PixelSizeMm;
			PixelSizeMm.x= attr.namedItem("PixelSizeMm").nodeValue().section(' ', 0, 0).toFloat();
			PixelSizeMm.y = attr.namedItem("PixelSizeMm").nodeValue().section(' ', 1, 1).toFloat();
			float viewportYMm = PixelSizeMm.y * ViewportPx.y;
			fov_= 2 * ((atanf(viewportYMm / (2 * FocalMm)))*180.0f / float(M_PI));
			std::cout << "fov: " << fov_ << std::endl;
			std::cout << "rotation_matrix:\n" << rotation_matrix << std::endl;
			std::cout << "trans_matrix:\n" << trans_matrix << std::endl;
		}
		node = node.nextSibling();
	}
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> view_matrix_;
	view_matrix_ = rotation_matrix * trans_matrix;
	Eigen::Matrix3f R = view_matrix_.inverse().topLeftCorner(3, 3);
	Eigen::Vector3f ea1 = R.eulerAngles(2, 1, 0);
	ea1 = ea1 / M_PI * 180;
	std::cout << ea1.x() << std::endl;
	std::cout << ea1.y() << std::endl;
	std::cout << ea1.z() << std::endl;
	std::cout << view_matrix_.inverse().topRightCorner(3, 1) << std::endl << std::endl;
	mvMatrix_origin_ = QMatrix4x4(view_matrix_.data());
	scale_ = 1;
	Matrix4fSetIdentity(&globalArcBall_->Transform);
	Matrix3fSetIdentity(&globalArcBall_->LastRot);
	Matrix3fSetIdentity(&globalArcBall_->ThisRot);
}

void OpenGLWindow::initializeGL()
{
	/*glFrontFace(GL_CCW);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);*/

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	bool success = shader_program_.addShaderFromSourceFile(QGLShader::Vertex, "../../src/Shader.vert");
	if (!success) {
		qDebug() << "shaderProgram addShaderFromSourceFile failed!" << shader_program_.log();
		return;
	}
	//success = shader_program_.addShaderFromSourceFile(QGLShader::Fragment, "cube.frag");
	success = shader_program_.addShaderFromSourceFile(QGLShader::Fragment, "../../src/Shader.frag");
	if (!success) {
		qDebug() << "shaderProgram addShaderFromSourceFile failed!" << shader_program_.log();
		return;
	}
	success = shader_program_.link();
	if (!success) {
		qDebug() << "shaderProgram link failed!" << shader_program_.log();
	}
}

void OpenGLWindow::resizeGL(int width, int height)
{
	w = width;
	h = height;
	globalArcBall_->setBounds(w, h);
	glViewport(0, 0, width, height);
	projMatrix_.setToIdentity();
	projMatrix_.perspective(fov_, (float)w/(float)h, 0.001, 100);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixf(projMatrix_.data());
}

void OpenGLWindow::paintGL()
{
	makeCurrent();
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_MULTISAMPLE);
	 
	QMatrix4x4 rotateMatrixg(globalArcBall_->Transform.M);
	rotateMatrixg = rotateMatrixg.transposed();
	/*QMatrix4x4 transMatrix;
	transMatrix.setToIdentity();
	transMatrix *= rotateMatrixg;
	transMatrix.scale(scale_);*/
	//transMatrix.translate(globalArcBall_->translate.s.X, globalArcBall_->translate.s.Y, globalArcBall_->translate.s.Z);
	//transMatrix.column(3) += transMatrix.column(0)*globalArcBall_->translate.s.X + transMatrix.column(1)*globalArcBall_->translate.s.Y;
	//std::cout << "translate: " << globalArcBall_->translate.s.X << " " << globalArcBall_->translate.s.Y << " " << globalArcBall_->translate.s.Z << std::endl;
	//::cout << "mvMatrix_translate: " << mvMatrix_.column(3).x() << " " << mvMatrix_.column(3).y() << " " << mvMatrix_.column(3).z() << std::endl;
	mvMatrix_ = mvMatrix_origin_*rotateMatrixg;
	mvMatrix_.translate(globalArcBall_->translate.s.X, globalArcBall_->translate.s.Z, globalArcBall_->translate.s.Y);
	mvMatrix_.scale(scale_);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf(mvMatrix_.data());
	shader_program_.bind();
	shader_program_.setUniformValue("sampler", 0);
	shader_program_.setUniformValue("mvMatrix", mvMatrix_);
	shader_program_.setUniformValue("projMatrix", projMatrix_);
	for (int i = 0; i < VAOs_.size(); i++) {
		textures_[i]->bind();
		VAOs_[i]->bind();
		glDrawArrays(GL_TRIANGLES, 0, counts_[i]);
		VAOs_[i]->release();
		textures_[i]->release();
	}
	shader_program_.release();
	//swapBuffers();
}

void OpenGLWindow::setAttributeToBuffer(tex::Model & model, std::string& out_prefix)
{
	model_ = &model;
	out_prefix_ = out_prefix;
	makeCurrent();
	ObjModel::Groups& groups = model.get_groups();
	ObjModel::Vertices& vertices = model.get_vertices();
	ObjModel::TexCoords& texcoords = model.get_texcoords();
	MaterialLib mtllib = model.get_material_lib();
	for (int i = 0; i < groups.size(); i++) {
		std::vector<ObjModel::Face>& faces = groups[i].faces;
		counts_.emplace_back(faces.size() * 3);
		std::vector<float> verts;
		for (int j = 0; j < faces.size(); j++) {
			for (int k = 0; k < 3; k++) {
				verts.push_back(vertices[faces[j].vertex_ids[k]][0]);
				verts.push_back(vertices[faces[j].vertex_ids[k]][1]);
				verts.push_back(vertices[faces[j].vertex_ids[k]][2]);
				verts.push_back(texcoords[faces[j].texcoord_ids[k]][0]);
				verts.push_back(texcoords[faces[j].texcoord_ids[k]][1]);
			}
		}
		//std::cout << "verts.size: " << verts.size() << std::endl;
		QOpenGLBuffer* vbo=new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
		QOpenGLVertexArrayObject* vao=new QOpenGLVertexArrayObject;
		//vbo = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
		//vao = new QOpenGLVertexArrayObject;
		vao->create();
		vao->bind();
		vbo->create();
		vbo->bind();
		vbo->setUsagePattern(QOpenGLBuffer::StaticDraw);
		vbo->allocate(verts.data(), sizeof(float) * verts.size());
		shader_program_.enableAttributeArray(0);
		shader_program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, 5 * sizeof(float));
		shader_program_.enableAttributeArray(1);
		shader_program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 2, 5 * sizeof(float));
		/*int size = vbo->size();
		std::cout << "VBOs_[0]->size() :" << size << std::endl;
		system("pause");*/
		vbo->release();
		vao->release();
		VBOs_.emplace_back(vbo);
		VAOs_.emplace_back(vao);
		
		mve::ByteImage::ConstPtr img = mtllib[i].diffuse_map;
		const uchar *pSrc = (const uchar*)img->get_data_pointer();
		QImage image(pSrc, img->width(), img->height(), QImage::Format_RGB888);
		QOpenGLTexture* texture = new QOpenGLTexture(image);
		texture->setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::Repeat);
		texture->setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::Repeat);

		texture->setMinificationFilter(QOpenGLTexture::Linear);
		texture->setMagnificationFilter(QOpenGLTexture::Linear);
		textures_.emplace_back(texture);
	}
	updateGL();
	/*VBOs_[0]->bind();
	int size = VBOs_[0]->size();
	std::cout << "VBOs_[0]->size() :" << size << std::endl;
	VBOs_[0]->release();
	std::cout << "VAOs_.size() :" << VAOs_.size() << std::endl;*/
}

void OpenGLWindow::mousePressEvent(QMouseEvent * event)
{
	/*if (event->buttons() & Qt::LeftButton) {
		lastPos_ = event->pos();
	}*/
	if (event->buttons() & Qt::LeftButton)
	{
		globalArcBall_->MousePt.s.X = event->pos().x();
		globalArcBall_->MousePt.s.Y = event->pos().y();
		globalArcBall_->isClicked = true;
		globalArcBall_->upstate();
		lastMouseButton = Qt::LeftButton;
		//lastPos_ = event->pos();
	}
	if (event->buttons() & Qt::RightButton)
	{
		globalArcBall_->MousePt.s.X = event->pos().x();
		globalArcBall_->MousePt.s.Y = event->pos().y();
		globalArcBall_->isRClicked = true;
		globalArcBall_->upstate();
		lastMouseButton = Qt::RightButton;
		
	}
	if (event->buttons() & Qt::MidButton)
	{
		//lastPos_ = event->pos();
		globalArcBall_->MousePt.s.X = event->pos().x();
		globalArcBall_->MousePt.s.Y = event->pos().y();
		globalArcBall_->isMClicked = true;
		globalArcBall_->upstate();
		lastMouseButton = Qt::MidButton;
	}
	updateGL();
}

void OpenGLWindow::mouseReleaseEvent(QMouseEvent * event)
{
	if (lastMouseButton == (int)Qt::LeftButton)
	{
		lastMouseButton = -1;
		globalArcBall_->MousePt.s.X = event->pos().x();
		globalArcBall_->MousePt.s.Y = event->pos().y();
		globalArcBall_->isClicked = false;
		globalArcBall_->upstate();
		//lastPos_ = event->pos();
	}
	if (lastMouseButton == (int)Qt::RightButton)
	{
		lastMouseButton = -1;
		globalArcBall_->MousePt.s.X = event->pos().x();
		globalArcBall_->MousePt.s.Y = event->pos().y();
		globalArcBall_->isRClicked = false;
		globalArcBall_->upstate();
	}
	if (lastMouseButton == (int)Qt::MidButton)
	{
		//lastPos_ = event->pos();
		lastMouseButton = -1;
		globalArcBall_->MousePt.s.X = event->pos().x();
		globalArcBall_->MousePt.s.Y = event->pos().y();
		globalArcBall_->isMClicked = false;
		globalArcBall_->upstate();
	}
	updateGL();
}

void OpenGLWindow::mouseMoveEvent(QMouseEvent * event)
{
	/*if (event->buttons() & Qt::LeftButton) {
		QPoint pos = event->pos();
		float xoffset = pos.x() - lastPos_.x();
		float yoffset = pos.y() - lastPos_.y();
		lastPos_ = pos;
		camera_->ProcessMouseMovement(xoffset, yoffset);
	}*/
	if (event->buttons() & Qt::LeftButton)
	{
		globalArcBall_->MousePt.s.X = event->pos().x();
		globalArcBall_->MousePt.s.Y = event->pos().y();
		globalArcBall_->isDragging = true;
		globalArcBall_->upstate();
		//lastPos_ = event->pos();
	}
	if (event->buttons() & Qt::RightButton)
	{
		globalArcBall_->MousePt.s.X = event->pos().x();
		globalArcBall_->MousePt.s.Y = event->pos().y();
		globalArcBall_->isZooming = true;
		globalArcBall_->upstate();
	}
	if (event->buttons() & Qt::MidButton)
	{
		//lastPos_ = event->pos();
		globalArcBall_->MousePt.s.X = event->pos().x();
		globalArcBall_->MousePt.s.Y = event->pos().y();
		globalArcBall_->isTranslating = true;
		globalArcBall_->upstate();
	}
	updateGL();
}

void OpenGLWindow::wheelEvent(QWheelEvent * event)
{
	if (event->delta() > 0) scale_ = scale_ + scale_ * 0.01;
	else scale_ = scale_ - scale_* 0.01;
	updateGL();
}

void OpenGLWindow::keyPressEvent(QKeyEvent * event)
{
	switch (event->key())
	{
	case Qt::Key_R:
		scale_ = 1;
		Matrix4fSetIdentity(&globalArcBall_->Transform);
		Matrix3fSetIdentity(&globalArcBall_->LastRot);
		Matrix3fSetIdentity(&globalArcBall_->ThisRot);
		break;
	case Qt::Key_I:
		if ((event->modifiers() == Qt::ControlModifier)) {
			loadCameraSlot();
		}
		break;
	case Qt::Key_S:
		if ((event->modifiers() == Qt::ControlModifier)) {
			std::cout << "\tSaving model... " << std::flush;
			tex::Model::save(*model_, out_prefix_);
			//std::cout<< out_prefix_ << std::endl;
			std::cout << "done." << std::endl;
		}
	}
	updateGL();
}


