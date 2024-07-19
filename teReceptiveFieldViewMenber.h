#pragma once

#include <QPoint>
class QStateMachine;
class QState;

namespace te {
	class ReceptiveFieldViewMenber
	{
	public:
		QPointF centerPoint;
		double ThrDRadius;
		double TwoDRadius;
		bool MaxState;
		bool circleVisible;
		int ReduceTimes;

		QStateMachine* stateMachine;
		QState* TwoDState;
		QState* ThrDState;
	};
}