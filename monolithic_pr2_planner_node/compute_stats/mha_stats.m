%directories = dir('/home/victor/ros/mpp_groovy/mpp/monolithic_pr2_planner_node/compute_stats/imha_all_set/');
directories = dir('/tmp/planning_stats/');
%for idx=3:size(directories,1)

success_rate = 0;
avg_time_ratio = 0;
avg_base_ratio = 0;
avg_ee_ratio = 0;

avg_cost_ratio = 0;
avg_expansion_ratio = 0;

range = 3:size(directories,1);
%range = 3:4

for idx=range
    cur_dir = directories(idx).name
    num = 10;
    %path = ['/home/victor/ros/mpp_groovy/mpp/monolithic_pr2_planner_node/compute_stats/imha_all_set/' cur_dir ]
    path = ['/tmp/planning_stats/' cur_dir ]
    rrt_stats = computeMethodStats([path '/rrt_'],num,0);
    smha_stats = computeMethodStats([path '/smha_'],num,1);
    prm_stats = computeMethodStats([path '/prm_'],num,0);
    rrtstar_stats = computeMethodStats([path '/rrtstar_'],num,0);
    ara_stats = computeMethodStats([path '/ara_'],num,1);
    imha_stats = computeMethodStats([path '/imha_'],num,1);





    %other_methods = [prm_stats rrt_stats rrtstar_stats ara_stats smha_stats];
    other_methods = [prm_stats rrt_stats rrtstar_stats imha_stats ara_stats ];
    smha_comparison = compareMethods(smha_stats,other_methods);

    % for each 'other' planner
    [path,planner_name,ext] = fileparts(smha_comparison.method.name)



    %ara_stats2 = computeMethodStatsSBPL([path '/ara_'],num);
    %smha_stats2 = computeMethodStatsSBPL([path '/smha_'],num);
    %sbpl_compare = compareMethodsSBPL(smha_stats2, [ara_stats2]);

    %if isnan(sbpl_compare.other(1).expansions.ratio_mean)
    %    sbpl_compare.other(1).expansions.ratio_mean = 0;
    %end
    %if isnan(sbpl_compare.other(1).cost.ratio_mean)
    %    sbpl_compare.other(1).cost.ratio_mean = 0;
    %end
    %if ~isfield(avg_expansion_ratio, planner_name)
    %    avg_expansion_ratio.(planner_name) = sbpl_compare.other(1).expansions.ratio_mean;
    %else
    %    avg_expansion_ratio.(planner_name) = avg_expansion_ratio.(planner_name) + sbpl_compare.other(1).expansions.ratio_mean/size(range,2);
    %end

    %if ~isfield(avg_cost_ratio, planner_name)
    %    avg_cost_ratio.(planner_name) = sbpl_compare.other(1).cost.ratio_mean;
    %else
    %    avg_cost_ratio.(planner_name) = avg_cost_ratio.(planner_name) + sbpl_compare.other(1).cost.ratio_mean/size(range,2);
    %end






    %if isnan(other(i).time.ratio_mean)
    %    other(i).time.ratio_mean = 0;
    %end

    if ~isfield(success_rate, planner_name)
        success_rate.(planner_name) = sum(smha_stats.base>0);
    else
        success_rate.(planner_name) = success_rate.(planner_name) + sum(smha_stats.base>0);
    end

    other = smha_comparison.other;
    for i=1:length(other_methods)
        [path,planner_name,ext] = fileparts(other_methods(i).name);
        if ~isfield(success_rate, planner_name)
            success_rate.(planner_name) = sum(other_methods(i).base>0);
        else
            success_rate.(planner_name) = success_rate.(planner_name) + sum(other_methods(i).base>0);
        end

        if isnan(other(i).time.ratio_mean)
            other(i).time.ratio_mean = 0;
        end

        if ~isfield(avg_time_ratio, planner_name)
            avg_time_ratio.(planner_name) = other(i).time.ratio_mean/size(range,2);
        else
            avg_time_ratio.(planner_name) = avg_time_ratio.(planner_name) + other(i).time.ratio_mean/size(range,2);
        end

        if isnan(other(i).base.ratio_mean)
            other(i).base.ratio_mean = 0;
        end
        other(i).name
        other(i).base.ratio_mean
        if ~isfield(avg_base_ratio, planner_name)
            avg_base_ratio.(planner_name) = other(i).base.ratio_mean/size(range,2);
        else
            avg_base_ratio.(planner_name) = avg_base_ratio.(planner_name) + other(i).base.ratio_mean/size(range,2);
        end

        if isnan(other(i).obj.ratio_mean)
            other(i).obj.ratio_mean = 0;
        end
        if ~isfield(avg_ee_ratio, planner_name)
            avg_ee_ratio.(planner_name) = other(i).obj.ratio_mean/size(range,2);
        else
            avg_ee_ratio.(planner_name) = avg_ee_ratio.(planner_name) + other(i).obj.ratio_mean/size(range,2);
        end

    end
    %other_methods = [prm_stats rrt_stats rrtstar_stats ara_stats smha_stats];
    %smha_comparison = compareMethods(smha_stats,other_methods);
end

success_rate
avg_time_ratio
avg_base_ratio
avg_ee_ratio
avg_cost_ratio
avg_expansion_ratio
