%directories = dir('/home/victor/ros/mpp_groovy/mpp/monolithic_pr2_planner_node/compute_stats/imha_all_set/');
stem = '/home/siddharth/Dropbox/Academics/CMU/Research/SBPL/Multiple_Hypothesis_Heuristics/automated_results/final_paper/ompl';
%stem = '/Users/vhwang/Desktop/stats/data/';
%stem = '/Users/vhwang/Desktop/stats/imha_all_set/';
%stem = '/Users/vhwang/Desktop/stats/meta_review_5/planning_stats/';
% stem = '/Users/vhwang/Desktop/stats/new/run2';
% stem = '/home/siddharth/Dropbox/Academics/CMU/Research/SBPL/Multiple_Hypothesis_Heuristics/automated_results/new_heur/victor/latest_stats';
directories = dir(stem);
%for idx=3:size(directories,1)


NUMBER_OF_TOTAL_PLANNING_REQUESTS = 0;

valid_dirs = [];
for i=1:length(directories)
    if regexp(directories(i).name, '[0-9]*')
        valid_dirs = [valid_dirs; i];
    end
end

range = valid_dirs;

cumulative_stats = {};
planners = {'smha_'; 'ara_'; 'imha_'; 'mpwa_'; 'mhg_reex_'; 'mhg_no_reex_'; 'ees_'; 'rrt_'; 'rrtstar_'; 'prm_'; 'rrtstarfirstsol_'};
% ompl_planners = {'rrt_'; 'rrtstar_'; 'imha_'; 'ara_'; 'prm_'; 'smha_'};
for planner_idx = 1:length(planners)
    cumulative_stats.(planners{planner_idx}) = {};
    cumulative_stats.(planners{planner_idx}).num_success = 0;
    cumulative_stats.(planners{planner_idx}).success_ratio = 0;

    cumulative_stats.(planners{planner_idx}).total_ee_length = 0;
    cumulative_stats.(planners{planner_idx}).ee_length_for_smha = 0;
    cumulative_stats.(planners{planner_idx}).ee_ratio = 0;

    cumulative_stats.(planners{planner_idx}).total_base_length = 0;
    cumulative_stats.(planners{planner_idx}).base_length_for_smha = 0;
    cumulative_stats.(planners{planner_idx}).base_ratio = 0;

    cumulative_stats.(planners{planner_idx}).total_time = 0;
    cumulative_stats.(planners{planner_idx}).total_time_for_smha = 0;
    cumulative_stats.(planners{planner_idx}).time_ratio = 0;

    cumulative_stats.(planners{planner_idx}).total_expands = 0;
    cumulative_stats.(planners{planner_idx}).total_expands_for_smha = 0;
    cumulative_stats.(planners{planner_idx}).expands_ratio = 0;

    cumulative_stats.(planners{planner_idx}).total_cost = 0;
    cumulative_stats.(planners{planner_idx}).total_cost_for_smha = 0;
    cumulative_stats.(planners{planner_idx}).cost_ratio = 0;
    cumulative_stats.(planners{planner_idx}).independent_total = 0;

    cumulative_stats.(planners{planner_idx}).num_successes_with_smha = 0;
end
total_num_env = 0;

smha_cost = 0;
ara_cost = 0;
imha_cost = 0;
mpwa_cost = 0;
mhg_reex_cost = 0;
mhg_no_reex_cost = 0;
ees_cost = 0;

for idx=1:length(valid_dirs)
    cur_dir = directories(valid_dirs(idx)).name;

    %look for the last env number that was generated, then ignore that one
    list_of_files = dir([stem '/' directories(valid_dirs(idx)).name]);
    counter = 1;
    valid_nums = [];
    for f_idx=1:size(list_of_files,1)
        if findstr(list_of_files(f_idx).name, 'env')
            str_idx = regexp(list_of_files(f_idx).name, '[0-9]*');
            valid_nums = [valid_nums; str2num(list_of_files(f_idx).name(str_idx:str_idx+1))];
            counter = counter + 1;
        end
    end
    if size(valid_nums,1) == 0
        continue;
    end
    valid_nums = sort(valid_nums);
    valid_nums = valid_nums(1:end);
    num = valid_nums(end) + 1;
    NUMBER_OF_TOTAL_PLANNING_REQUESTS = NUMBER_OF_TOTAL_PLANNING_REQUESTS + num;
    % num = 10;
    fprintf('computing stats over %d experiments\n', num);
    %path_ = ['/home/victor/ros/mpp_groovy/mpp/monolithic_pr2_planner_node/compute_stats/imha_all_set/' cur_dir ]
    path_ = [stem '/' cur_dir ];
    %rrt_stats = computeMethodStats([path_ '/rrt_'],num,0);
    %rrtstar_stats = computeMethodStats([path_ '/rrtstar_'],num,0);
    %prm_stats = computeMethodStats([path_ '/prm_'],num,0);
    %rrtstarfirstsol_stats = computeMethodStats([path_ '/rrtstarfirstsol_'],num,0);
    %smha_stats = computeMethodStats([path_ '/smha_'],num,1);
    %ara_stats = computeMethodStats([path_ '/ara_'],num,1);
    %imha_stats = computeMethodStats([path_ '/imha_'],num,1);
    %other_methods = [rrt_stats prm_stats rrtstar_stats imha_stats ara_stats];
    %smha_comparison = compareMethods(smha_stats,other_methods);

    isSBPL = 1;
    smha_stats = computeMethodStats([path_ '/smha_'],num,isSBPL);
    ara_stats = computeMethodStats([path_ '/ara_'],num,isSBPL);
    imha_stats = computeMethodStats([path_ '/imha_'],num,isSBPL);
    mpwa_stats = computeMethodStats([path_ '/mpwa_'],num,isSBPL);
    mhg_reex_stats = computeMethodStats([path_ '/mhg_reex_'],num,isSBPL);
    mhg_no_reex_stats = computeMethodStats([path_ '/mhg_no_reex_'],num,isSBPL);
    ees_stats = computeMethodStats([path_ '/ees_'],num,isSBPL);
    other_methods = [mpwa_stats mhg_no_reex_stats mhg_reex_stats ees_stats imha_stats ara_stats];

    isSBPL = 0;
    rrt_stats = computeMethodStats([path_ '/rrt_'],num,isSBPL);
    rrtstar_stats = computeMethodStats([path_ '/rrtstar_'],num,isSBPL);
    prm_stats = computeMethodStats([path_ '/prm_'],num,isSBPL);
    rrtstarfirstsol_stats = computeMethodStats([path_ '/rrtstarfirstsol_'],num,isSBPL);
    other_methods = [other_methods rrt_stats rrtstar_stats prm_stats rrtstarfirstsol_stats];

    smha_comparison = compareMethods(smha_stats,other_methods);

    smha_cost = smha_cost + sum(smha_stats.cost(find(smha_stats.cost>0)));
    ara_cost = ara_cost + sum(ara_stats.cost(find(ara_stats.cost>0)));
    imha_cost = imha_cost + sum(imha_stats.cost(find(imha_stats.cost>0)));
    mpwa_cost = mpwa_cost + sum(mpwa_stats.cost(find(mpwa_stats.cost>0)));
    mhg_reex_cost = mhg_reex_cost + sum(mhg_reex_stats.cost(find(mhg_reex_stats.cost>0)));
    mhg_no_reex_cost = mhg_no_reex_cost + sum(mhg_no_reex_stats.cost(find(mhg_no_reex_stats.cost>0)));
    ees_cost = ees_cost + sum(ees_stats.cost(find(ees_stats.cost>0)));


    % for each 'other' planner
    [path_,planner_name,ext] = fileparts(smha_comparison.method.name);

    % for each folder of experiments, compute how successes the base (smha) had
    cumulative_stats.(planner_name).num_success = cumulative_stats.(planner_name).num_success + sum(smha_stats.base>0);

    other = smha_comparison.other;
    for i=1:length(other_methods)
        current_comparison = other(i);
        [path_,planner_name,ext] = fileparts(other_methods(i).name);
        cur_num_success = other(i).num_success;
        % the total number of successes each planner had
        cumulative_stats.(planner_name).num_success = cumulative_stats.(planner_name).num_success + sum(other_methods(i).base>0);
        % let's keep track of the total planning time for all experiments where
        % the base planner and this "other" planner succeeded

        num_both_successes = current_comparison.num_success;
        cumulative_stats.(planner_name).num_successes_with_smha = cumulative_stats.(planner_name).num_successes_with_smha + num_both_successes;
        m_total_time = current_comparison.time.m_mean*num_both_successes;
        o_total_time = current_comparison.time.o_mean*num_both_successes;

        % need this because these values will be NaN when there are no successes
        if cur_num_success ~= 0
            cumulative_stats.(planner_name).total_time = cumulative_stats.(planner_name).total_time + o_total_time;
            cumulative_stats.(planner_name).total_time_for_smha = cumulative_stats.(planner_name).total_time_for_smha + m_total_time;

            % let's keep track of the total base length for all experiments where
            % the base planner and this "other" planner succeeded
            m_total_base_length = current_comparison.base.m_mean*num_both_successes;
            o_total_base_length = current_comparison.base.o_mean*num_both_successes;
            cumulative_stats.(planner_name).total_base_length = cumulative_stats.(planner_name).total_base_length + o_total_base_length;
            cumulative_stats.(planner_name).base_length_for_smha = cumulative_stats.(planner_name).base_length_for_smha + m_total_base_length;

            % let's keep track of the total base length for all experiments where
            % the base planner and this "other" planner succeeded
            m_total_ee_length = current_comparison.obj.m_mean*num_both_successes;
            o_total_ee_length = current_comparison.obj.o_mean*num_both_successes;
            cumulative_stats.(planner_name).total_ee_length = cumulative_stats.(planner_name).total_ee_length + o_total_ee_length;
            cumulative_stats.(planner_name).ee_length_for_smha = cumulative_stats.(planner_name).ee_length_for_smha + m_total_ee_length;

            % if isSBPL
                m_total_cost = current_comparison.cost.m_mean*num_both_successes;
                o_total_cost = current_comparison.cost.o_mean*num_both_successes;
                cumulative_stats.(planner_name).total_cost = cumulative_stats.(planner_name).total_cost + o_total_cost;
                cumulative_stats.(planner_name).total_cost_for_smha = cumulative_stats.(planner_name).total_cost_for_smha + m_total_cost;

                m_total_expands = current_comparison.expands.m_mean*num_both_successes;
                o_total_expands = current_comparison.expands.o_mean*num_both_successes;
                cumulative_stats.(planner_name).total_expands = cumulative_stats.(planner_name).total_expands + o_total_expands;
                cumulative_stats.(planner_name).total_expands_for_smha = cumulative_stats.(planner_name).total_expands_for_smha + m_total_expands;
            % end
        end
    end
end
for planner_idx = 1:length(planners)
    planner = planners{planner_idx};


    % i've thrown this in here due to laziness. the data set i'm using right now
    % has 40 planning requests - 04/17/2014 @ 4:11pm
    cumulative_stats.(planner).success_ratio = cumulative_stats.(planner).num_success/NUMBER_OF_TOTAL_PLANNING_REQUESTS;
    cumulative_stats.(planner).ee_ratio = cumulative_stats.(planner).total_ee_length/cumulative_stats.(planner).ee_length_for_smha;
    cumulative_stats.(planner).base_ratio = cumulative_stats.(planner).total_base_length/cumulative_stats.(planner).base_length_for_smha;
    cumulative_stats.(planner).time_ratio = cumulative_stats.(planner).total_time /cumulative_stats.(planner).total_time_for_smha;
    cumulative_stats.(planner).cost_ratio = cumulative_stats.(planner).total_cost /cumulative_stats.(planner).total_cost_for_smha;
    cumulative_stats.(planner).expands_ratio = cumulative_stats.(planner).total_expands /cumulative_stats.(planner).total_expands_for_smha;

    disp(planner);
    disp(['Number of requests: ', num2str(NUMBER_OF_TOTAL_PLANNING_REQUESTS)]);
    disp(['Number of total successes: ', num2str(cumulative_stats.(planner).num_success)]);
    disp(['Success rate over all planning requests: ', num2str(cumulative_stats.(planner).success_ratio)]);
    disp(['Number of trials where both planners succeeded: ', num2str(cumulative_stats.(planner).num_successes_with_smha)])
    disp(['End effector ratio (other/smha): ', num2str(cumulative_stats.(planner).ee_ratio)]);
    disp(['Base distance ratio (other/smha): ', num2str(cumulative_stats.(planner).base_ratio)]);
    disp(['Time ratio (other/smha): ', num2str(cumulative_stats.(planner).time_ratio)])
    % if isSBPL
        disp(['Expands ratio (other/smha): ', num2str(cumulative_stats.(planner).expands_ratio)]);
        disp(['Cost ratio (other/smha): ', num2str(cumulative_stats.(planner).cost_ratio)]);
        disp(['Cumulative Avg Cost (other): ', num2str(cumulative_stats.(planner).total_cost/cumulative_stats.(planner).num_successes_with_smha)]);
    % end
    fprintf('\n');
end
disp(['smha average cost: ', num2str(smha_cost/cumulative_stats.smha_.num_success)]);
disp(['ara average cost: ', num2str(ara_cost/cumulative_stats.ara_.num_success)]);
disp(['imha average cost: ', num2str(imha_cost/cumulative_stats.imha_.num_success)]);
disp(['mpwa average cost: ', num2str(mpwa_cost/cumulative_stats.mpwa_.num_success)]);
disp(['mhg_reex average cost: ', num2str(mhg_reex_cost/cumulative_stats.mhg_reex_.num_success)]);
disp(['mhg_no_reex average cost: ', num2str(mhg_no_reex_cost/cumulative_stats.mhg_no_reex_.num_success)]);
disp(['ees average cost: ', num2str(ees_cost/cumulative_stats.ees_.num_success)]);

