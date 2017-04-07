// Bio IK for ROS
// Philipp Ruppel

#pragma once

#include "ik_base.h"

#include "ik_evolution_1.h"

namespace bio_ik
{



// pseudoinverse jacobian solver
// (mainly for testing RobotFK_Jacobian::computeJacobian)
template<class BASE>
struct IKJacobianBase : BASE
{
    using BASE::modelInfo;
    using BASE::model;
    using BASE::params;
    using BASE::computeFitness;
    using BASE::getGenes;
    using BASE::active_variables;
    using BASE::request;
    
    std::vector<Frame> tipObjectives;
    
    Eigen::VectorXd tip_diffs;
    Eigen::VectorXd joint_diffs;
    Eigen::MatrixXd jacobian;
    std::vector<Frame> tip_frames_temp;

    IKJacobianBase(const IKParams& p) : BASE(p)
    {
    }
    
    void initialize(const IKRequest& request)
    {
        BASE::initialize(request);
        tipObjectives.resize(request.tip_link_indices.size());
        for(auto& goal : request.goals)
            tipObjectives[goal.tip_index] = goal.frame;
    }
    
    void optimizeJacobian(std::vector<double>& solution)
    {
        FNPROFILER();

        int tip_count = request.tip_link_indices.size();
        tip_diffs.resize(tip_count * 6);
        joint_diffs.resize(getGenes().size());

        // compute fk
        model.applyConfiguration(solution);
        
        double translational_scale = 1;
        double rotational_scale = 1;
        
        // compute goal diffs
        tip_frames_temp = model.getTipFrames();
        for(int itip = 0; itip < tip_count; itip++)
        {
            auto twist = frameTwist(tip_frames_temp[itip], tipObjectives[itip]);
            tip_diffs(itip * 6 + 0) = twist.vel.x() * translational_scale;
            tip_diffs(itip * 6 + 1) = twist.vel.y() * translational_scale;
            tip_diffs(itip * 6 + 2) = twist.vel.z() * translational_scale;
            tip_diffs(itip * 6 + 3) = twist.rot.x() * rotational_scale;
            tip_diffs(itip * 6 + 4) = twist.rot.y() * rotational_scale;
            tip_diffs(itip * 6 + 5) = twist.rot.z() * rotational_scale;
        }
        
        // compute jacobian
        {
            model.computeJacobian(active_variables, jacobian);
            int icol = 0;
            for(auto ivar : getGenes())
            {
                for(size_t itip = 0; itip < tip_count; itip++)
                {
                    jacobian(itip * 6 + 0, icol) *= translational_scale;
                    jacobian(itip * 6 + 1, icol) *= translational_scale;
                    jacobian(itip * 6 + 2, icol) *= translational_scale;
                    jacobian(itip * 6 + 3, icol) *= rotational_scale;
                    jacobian(itip * 6 + 4, icol) *= rotational_scale;
                    jacobian(itip * 6 + 5, icol) *= rotational_scale;
                }
                icol++;
            }
        }

        // get pseudoinverse and multiply
        joint_diffs = jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(tip_diffs);
        //joint_diffs = (jacobian.transpose() * jacobian).ldlt().solve(jacobian.transpose() * tip_diffs);

        // apply joint deltas and clip
        {
            int icol = 0;
            for(auto ivar : active_variables)
            {
                auto v = solution[ivar] + joint_diffs(icol);
                if(!std::isfinite(v)) continue;
                v = modelInfo.clip(v, ivar);
                solution[ivar] = v;
                icol++;
            }
        }
    }
};






// simple gradient descent
template<int if_stuck, size_t threads>
struct IKGradientDescent : IKBase
{
    std::vector<double> solution, best_solution, gradient, temp;
    bool reset;

    IKGradientDescent(const IKParams& p) : IKBase(p)
    {
    }
    
    void initialize(const IKRequest& request)
    {
        IKBase::initialize(request);
        solution = request.initial_guess;
        if(thread_index > 0)
            for(auto& vi : active_variables)
                solution[vi] = random(modelInfo.getMin(vi), modelInfo.getMax(vi));
        best_solution = solution;
        reset = false;
    }
    
    const std::vector<double>& getSolution() const
    {
        return best_solution;
    }
    
    void step()
    {
        // random reset if stuck
        if(reset)
        {
            reset = false;
            for(auto& vi : active_variables)
                solution[vi] = random(modelInfo.getMin(vi), modelInfo.getMax(vi));
        }
    
        // compute gradient direction
        temp = solution;
        double jd = 0.0001;
        gradient.resize(solution.size(), 0);
        for(auto ivar : active_variables)
        {
            temp[ivar] = solution[ivar] - jd;
            double p1 = computeFitness(temp);
            
            temp[ivar] = solution[ivar] + jd;
            double p3 = computeFitness(temp);
            
            temp[ivar] = solution[ivar];
            
            gradient[ivar] = p3 - p1;
        }
        
        // normalize gradient direction
        double sum = 0.0001;
        for(auto ivar : active_variables)
            sum += fabs(gradient[ivar]);
        double f = 1.0 / sum * jd;
        for(auto ivar : active_variables)
            gradient[ivar] *= f;
    
        // initialize line search
        temp = solution;
        
        for(auto ivar : active_variables) temp[ivar] = solution[ivar] - gradient[ivar];
        double p1 = computeFitness(temp);
        
        for(auto ivar : active_variables) temp[ivar] = solution[ivar];
        double p2 = computeFitness(temp);
        
        for(auto ivar : active_variables) temp[ivar] = solution[ivar] + gradient[ivar];
        double p3 = computeFitness(temp);

        // linear step size estimation
        double cost_diff = (p3 - p1) * 0.5;
        double joint_diff = p2 / cost_diff;
        
        // apply optimization step
        // (move along gradient direction by estimated step size)
        for(auto ivar : active_variables)
            temp[ivar] = modelInfo.clip(solution[ivar] - gradient[ivar] * joint_diff, ivar);
        
        // has solution improved?
        if(computeFitness(temp) < computeFitness(solution)) 
        {
            // solution improved -> accept solution
            solution = temp; 
        }
        else
        {
            if(if_stuck == 'r')
            {
                // reset if stuck
                reset = true;
            }
            if(if_stuck == 'c')
            {
                // always accept solution and continue
                solution = temp;
            }
        }
        
        // update best solution
        if(computeFitness(solution) < computeFitness(best_solution))
            best_solution = solution;
    }
    
    size_t concurrency() const { return threads; }
};

static IKFactory::Class<IKGradientDescent<' ', 1>> gd("gd");
static IKFactory::Class<IKGradientDescent<' ', 2>> gd_2("gd_2");
static IKFactory::Class<IKGradientDescent<' ', 4>> gd_4("gd_4");
static IKFactory::Class<IKGradientDescent<' ', 8>> gd_8("gd_8");

static IKFactory::Class<IKGradientDescent<'r', 1>> gd_r("gd_r");
static IKFactory::Class<IKGradientDescent<'r', 2>> gd_2_r("gd_r_2");
static IKFactory::Class<IKGradientDescent<'r', 4>> gd_4_r("gd_r_4");
static IKFactory::Class<IKGradientDescent<'r', 8>> gd_8_r("gd_r_8");

static IKFactory::Class<IKGradientDescent<'c', 1>> gd_c("gd_c");
static IKFactory::Class<IKGradientDescent<'c', 2>> gd_2_c("gd_c_2");
static IKFactory::Class<IKGradientDescent<'c', 4>> gd_4_c("gd_c_4");
static IKFactory::Class<IKGradientDescent<'c', 8>> gd_8_c("gd_c_8");





// only pseudoinverse jacobian
template<size_t threads>
struct IKJacobian : IKJacobianBase<IKBase>
{
    using IKBase::initialize;
    std::vector<double> solution;
    IKJacobian(const IKParams& p) : IKJacobianBase<IKBase>(p)
    {
    }
    void initialize(const IKRequest& request)
    {
        IKJacobianBase<IKBase>::initialize(request);
        solution = request.initial_guess;
        if(thread_index > 0)
            for(auto& vi : active_variables)
                solution[vi] = random(modelInfo.getMin(vi), modelInfo.getMax(vi));
    }
    const std::vector<double>& getSolution() const
    {
        return solution;
    }
    void step()
    {
        optimizeJacobian(solution);
    }
    size_t concurrency() const { return threads; }
};
static IKFactory::Class<IKJacobian<1>> jac("jac");
static IKFactory::Class<IKJacobian<2>> jac_2("jac_2");
static IKFactory::Class<IKJacobian<4>> jac_4("jac_4");
static IKFactory::Class<IKJacobian<8>> jac_8("jac_8");



// experiment: combining evolution and pseudoinverse jacobian
// (fail. worse than bio2 evolution only.)
template<class BASE>
struct IKHybrid : IKJacobianBase<BASE>
{
    using BASE::computeFitness;
    using IKJacobianBase<BASE>::optimizeJacobian;
    std::vector<double> best_solution, optimized_solution;
    double best_fitness, optimized_fitness;
    IKHybrid(const IKParams& p) : IKJacobianBase<BASE>(p)
    {
    }
    void initialize(const IKRequest& request)
    {
        BASE::initialize(request);
        best_solution = request.initial_guess;
        best_fitness = computeFitness(best_solution);
        optimized_solution = best_solution;
        optimized_fitness = best_fitness;
    }
    const std::vector<double>& getSolution()
    {
        return best_solution;
    }
    void step()
    {
        FNPROFILER();
        BASE::step();
        {
            auto& current_solution = BASE::getSolution();
            double current_fitness = computeFitness(current_solution);
            if(current_fitness < best_fitness) best_fitness = current_fitness, best_solution = current_solution;   
            if(current_fitness < optimized_fitness) optimized_fitness = current_fitness, optimized_solution = current_solution;   
        }
        {
            optimizeJacobian(optimized_solution);
            double optimized_fitness = computeFitness(optimized_solution);
            if(optimized_fitness < best_fitness) best_fitness = optimized_fitness, best_solution = optimized_solution;
        }
    }
    size_t concurrency() const { return 4; }
};
//static IKFactory::Class<IKHybrid<IKEvolution>> bio1jac("bio1jac");
static IKFactory::Class<IKHybrid<IKEvolution2>> bio2jac("bio2jac");



}









