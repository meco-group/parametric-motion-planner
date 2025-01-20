#include <vector>
#include <string>
#include <sstream>

#include "core/helper_types.hpp"
#include "core/corridor.hpp"

///////////////////////////
// MOTION PLANNER EVENTS //
///////////////////////////

class PlannerEvent{
    public:
        PlannerEvent(int print_level) : print_level_(print_level){};

        virtual ~PlannerEvent() = default;

        virtual std::string Serialize(bool compact=false) const = 0;

        virtual int GetPrintLevel() const {return print_level_;};

    protected:
        const int print_level_;
};

class PlannerCalledEvent : public PlannerEvent{
    public:
        PlannerCalledEvent(Point2D<double> const &start,
                           Point2D<double> const &start_vel,
                           Point2D<double> const &dest)
                           : PlannerEvent(1), start_(start), 
                             start_vel_(start_vel), dest_(dest){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            out << "Planner called";
            if (!compact){
                out << " to plan from " << start_ << " to " << dest_ << " with start velocity " << start_vel_;
            }
            return out.str();
        }

    private:
        const Point2D<double> start_;
        const Point2D<double> start_vel_;
        const Point2D<double> dest_;
};

class PlannerCalledSafelyEvent : public PlannerEvent{
    public:
        PlannerCalledSafelyEvent(Point2D<double> const &start,
                                 Point2D<double> const &start_vel,
                                 Point2D<double> const &dest)
                                 : PlannerEvent(0), start_(start), 
                                   start_vel_(start_vel), dest_(dest){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            if (compact){out << std::endl;}
            out << "Planner called safely";
            if (!compact){
                out << " to plan from " << start_ << " to " << dest_ << " with start velocity " << start_vel_;
            }
            return out.str();
        }

    private:
        const Point2D<double> start_;
        const Point2D<double> start_vel_;
        const Point2D<double> dest_;
};

class MethodSpecificPlanningStarted : public PlannerEvent{
    public:
        MethodSpecificPlanningStarted(std::string planner_name)
        : PlannerEvent(1), method_(planner_name){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            out << "\tPlanning started using method: " << method_;
            return out.str();
        }

    private:
        const std::string method_;
};

class EmergencyBrakingPlanningStarted : public PlannerEvent{
    public:
        EmergencyBrakingPlanningStarted(Point2D<double> const &start,
                                        Point2D<double> const &start_vel)
                                        : PlannerEvent(0), start_(start), 
                                          start_vel_(start_vel){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            out << "Emergency braking planning started";
            if (!compact){
                out << " from " << start_ << " with start velocity " << start_vel_;
            }
            return out.str();
        }

    private:
        const Point2D<double> start_;
        const Point2D<double> start_vel_;
};

class EmergencyBrakingPlanningFailed : public PlannerEvent{
    public:
        EmergencyBrakingPlanningFailed(std::string reason)
        : PlannerEvent(0), reason_(reason){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            out << "Emergency braking planning failed";
            if (!compact){
                out << ": " << reason_;
            }
            return out.str();
        }

    private:
        const std::string reason_;
};

class ConcatenatedSectionsPlanningStarted : public PlannerEvent{
    public:
        ConcatenatedSectionsPlanningStarted(Point2D<double> const &start,
                                            Point2D<double> const &start_vel,
                                            Point2D<double> const &dest)
                                            : PlannerEvent(1), start_(start), 
                                              start_vel_(start_vel),
                                              dest_(dest){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            out << "Planning using concatenated sections";
            if (!compact){
                out << " from " << start_ << " to " << dest_ << " with start velocity " << start_vel_;
            }
            return out.str();
        }

    private:
        const Point2D<double> start_;
        const Point2D<double> start_vel_;
        const Point2D<double> dest_;
};

class ConcatenatedSectionPlanningStarted : public PlannerEvent{
    public:
        ConcatenatedSectionPlanningStarted(Point2D<double> const &start,
                                           Point2D<double> const &start_vel,
                                           Point2D<double> const &dest)
                                           : PlannerEvent(1), start_(start), 
                                             start_vel_(start_vel),
                                             dest_(dest){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            out << "\tPlanning concatenated section";
            if (!compact){
                out << " from " << start_ << " to " << dest_ << " with start velocity " << start_vel_;
            }
            return out.str();
        }

    private:
        const Point2D<double> start_;
        const Point2D<double> start_vel_;
        const Point2D<double> dest_;
};

class PlannerFailedEvent : public PlannerEvent {
    public:
        PlannerFailedEvent() : PlannerEvent(1){};

        std::string Serialize(bool compact=false) const override {
            return "Planner failed to find a solution";
        }
};

class PlannerExitedEvent : public PlannerEvent{
    public:
        PlannerExitedEvent() : PlannerEvent(1){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            out << "Planner exited" << std::endl;
            return out.str();
        }
};

class SafelyPlannerExitedEvent : public PlannerEvent {
    public:
        SafelyPlannerExitedEvent() : PlannerEvent(1){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            out << "Safely planner exited" << std::endl;
            return out.str();
        }
};

class PlannerExceptionCaught : public PlannerEvent{
    public:
        PlannerExceptionCaught(std::string exception_message)
        : PlannerEvent(1), exception_message_(exception_message){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            out << "\tCaught exception: " << exception_message_;
            return out.str();
        }

    private:
        const std::string exception_message_;
};

class UpdatedCorridorsEvent : public PlannerEvent{
    public:
        UpdatedCorridorsEvent(CorridorSequence const &sequence)
        : PlannerEvent(2), sequence_(sequence){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            out << "\tUpdated corridors" << std::endl;
            if (!compact){
                out << ": " << sequence_;
            }
            return out.str();
        }

    private:
        const CorridorSequence sequence_;
};

class AddedAdditionalConstraintsEvent : public PlannerEvent {
    public:
        AddedAdditionalConstraintsEvent(std::set<int> const &add_list)
        : PlannerEvent(3), add_list_(add_list){};

        std::string Serialize(bool compact=false) const override {
            std::ostringstream out;
            out << "\tAdded additional constraints to corridors";
            if (!compact){
                out << ": ";
                for (auto &corridor_idx : add_list_){
                    out << corridor_idx << " ";
                }
            }
            return out.str();
        }

    private:
        const std::set<int> add_list_;
};

class EliminatedSubOptimalParametrizationEvent : public PlannerEvent{
    public:
        EliminatedSubOptimalParametrizationEvent(bool made_modification)
        : PlannerEvent(3), made_modification_(made_modification){};

        std::string Serialize(bool compact=false) const override {
            if (made_modification_){
                return "\tEliminated sub-optimal parametrization";
            } else {
                return "\tAttempted to eliminate sub-optimal parametrization";
            }
        }

    private:
        const bool made_modification_;
};

////////////////////////////
// PARAMETRIZATION EVENTS //
////////////////////////////

// class SolvingParametrization : public PlannerEvent{
//     public:
//         SolvingParametrization(std::string code)
//         : code_(code){};

//         std::string Serialize(bool compact=false) const override {
//             return "Solving parametrization with code " + code_;
//         }

//     private:
//         const std::string code_;
// }


////////////
// LOGGER //
////////////
class PlannerLogger{
    public:
        PlannerLogger(){};

        template <typename T, typename = std::enable_if_t<std::is_base_of_v<PlannerEvent, T>>>
        void LogEvent(T&& event){
            auto event_ptr = std::make_shared<std::decay_t<T>>(std::forward<T>(event));
            events_.push_back(event_ptr);
        }

        void PrintLog() const {
            std::cout << std::endl << "========= LOGGER OUTPUT =========" << std::endl;
            for (auto &event : events_){
                if (event->GetPrintLevel() <= print_level_){
                    std::cout << event->Serialize(compact_) << std::endl;
                }
            }
            std::cout << "=================================" << std::endl;
        }

        void Reset(){ events_.clear();};

        int GetPrintLevel() const { return print_level_;};
        void SetPrintLevel(int print_level) { print_level_ = print_level;};

        void SetCompact(bool compact) { compact_ = compact;};

    private:
        std::vector<std::shared_ptr<PlannerEvent>> events_;
        int print_level_ = 1;
        bool compact_ = true;
};