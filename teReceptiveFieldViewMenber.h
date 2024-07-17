#pragma once

#include <QPoint>
class QStateMachine;
class QState;

namespace te {
	class ReceptiveFieldViewMenber
	{
	public:
		QPointF centerPoint;
		double ThrDradius;
		double TwoDradius;
		bool MaxState;
		bool circleVisible;
		int ReduceTimes;

		QStateMachine* stateMachine;
		QState* TwoDState;
		QState* ThrDState;
	};
}