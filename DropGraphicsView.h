#pragma once

#include <QLabel>

class DropGraphicsView  : public QLabel
{
	Q_OBJECT

public:
	DropGraphicsView(QWidget *parent=nullptr);
	~DropGraphicsView();
protected:
	void dragEnterEvent(QDragEnterEvent* event) override;
	void dropEvent(QDropEvent* event) override;

signals:
	void pointSignal(const QVariant& pointVariant);
	void labelSignal(const QString& labelText);
};
