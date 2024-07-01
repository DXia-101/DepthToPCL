#pragma once

#include "teBaseDevelopmentKit.h"
#include "teTraining.h"
#include "teAiExTypes.h"

#include "teObjectTreeWidget.h"
#include "teObjectTreeWidgetItem.h"

#include <QObject>

TE_BEGIN_NAMESPACE

class TrainParamRegister
{
public:
	TrainParamRegister();

public:
	int TrainBatchSize;
	int PatchWidth;
	int PatchHeight;
	int receptiveField;
	int trainIterCnt;
	int saveFrequency;
	te::LocateType eLocateType;
	int locateSide;
	StdU8String netName;
	std::vector<ImageDesc> sampleDesc;
	int DeviceID;
	bool AutomaticStop;
};


class TrainNetComboxItem : public QObject, public AbstractStringItem
{
	Q_OBJECT

public:

	TrainNetComboxItem(const QString& key) :AbstractStringItem(key) {}

	QWidget* createItemWidget(const StdU8String& str)override
	{
		QComboBox* pCombox = new QComboBox;
		ToolType toolType = ToolType::E_PixelDetect_Tool;
		std::vector<std::string> strings = Training::getNetworkList(toolType);
		QStringList netNames;
		for (const std::string& netName : strings) {
			netNames << QString::fromStdString(netName);
		}
		pCombox->addItems(netNames);

		pCombox->setCurrentText(QString::fromStdString(str));

		connect(pCombox, &QComboBox::currentTextChanged, this, [this, pCombox]()
			{
				emit sig_ValueChange();
			});

		return pCombox;
	}

	bool getValue(QWidget* pWidget, StdU8String* pValue)override
	{
		QComboBox* pCombox = static_cast<QComboBox*>(pWidget);
		*pValue = pCombox->currentText().toStdString();
		return true;
	}

	void setValue(QWidget* pWidget, const StdU8String& str)override
	{
		QComboBox* pEdit = static_cast<QComboBox*>(pWidget);
		pEdit->setCurrentText(QString::fromStdString(str));
	}

signals:
	void sig_ValueChange();
};


class TrainParamWriter :public ObjectTreeWidgetWriter
{
	Q_OBJECT

public:

	QPair<AbstractStringItem*, QWidget*> createStringItem(const QString& key, const StdU8String& str, QStack<ObjectTreeWidgetItem*> visitingNode)override
	{
		if (key != "netName") {
			return ObjectTreeWidgetWriter::createStringItem(key, str, visitingNode);
		}

		TrainNetComboxItem* pItem = new TrainNetComboxItem(key);
		QWidget* pWidget = pItem->createItemWidget(str);
		connect(pItem, &TrainNetComboxItem::sig_ValueChange, this, [this, pItem]()
			{
				emit sig_ItemChange(pItem);
			});

		return { pItem ,pWidget };
	}

};


TE_END_NAMESPACE