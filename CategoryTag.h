#pragma once

#include <QLabel>

class CategoryTag  : public QLabel
{
	Q_OBJECT

public:
	CategoryTag(const QString& text, bool status, QWidget* parent = nullptr);
	~CategoryTag();
protected:
	void mousePressEvent(QMouseEvent* event) override;
	void showContextMenu(const QPoint& pos);
	void deleteLabel();

private:
	bool isBase;
};
