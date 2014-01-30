function comparison = computeStats(path)
  num = 10;

  prm_stats = computeMethodStats([path '/prm_'],num,0)
  rrt_stats = computeMethodStats([path '/rrt_'],num,0)
  rrtstar_stats = computeMethodStats([path '/rrtstar_'],num,0)
  ara_stats = computeMethodStats([path '/ara_'],num,1)
  keyboard
  other_methods = [prm_stats rrt_stats rrtstar_stats];
  %other_methods = [cbirrt_stats multi_ompl_stats];

  comparison = compareMethods(ara_stats,other_methods);

  %displayComparison(comparison);
end

function stats = computeMethodStats(folder_name,num,sbpl)
  base = [];
  spine = [];
  obj = [];
  arm_sqr = [];
  arm_abs = [];
  time = [];
  for i=1:num
    path_filename = [folder_name num2str(i-1,'%02d') '.path'];
    stat_filename = [folder_name num2str(i-1,'%02d') '.stats'];
    if exist(path_filename, 'file')~=2 || exist(stat_filename, 'file')~=2
      base(i) = -1;
      spine(i) = -1;
      obj(i) = -1;
      arm_abs(i) = -1;
      arm_sqr(i) = -1;
      time(i) = -1;
      continue;
    end
    raw_path = load(path_filename);
    %read 'sub-trajectories' that make up the full path
    base_path = raw_path(:,1:3);
    spine_path = raw_path(:,4);
    arm_path = raw_path(:,5:11);
    obj_path = raw_path(:,12:15);

    base(i) = sum(sqrt(sum(diff(base_path(:,1:2)).^2,2)));
    spine(i) = sum(sqrt(sum(diff(spine_path).^2,2)));
    obj(i) = sum(sqrt(sum(diff(obj_path(:,1:3)).^2,2)));

    %compute arm path
    arm_diff = [];
    for j=2:size(arm_path,1)
       for k=1:size(arm_path,2)
         ang1 = mod(arm_path(j-1,k), 2*pi);
         ang2 = mod(arm_path(j,k), 2*pi);
         ang = mod(ang2-ang1, 2*pi);
         if ang > pi
           ang = -(2.0*pi - ang);
         end
         arm_diff(j-1,k) = ang;
       end
    end
    arm_abs(i) = sum(sqrt(sum(abs(arm_diff),2)));
    arm_sqr(i) = sum(sqrt(sum(arm_diff.^2,2)));

    %get plan times
    raw_stats = load(stat_filename);
    if sbpl
      time(i) = raw_stats(1,4);
    else
      time(i) = raw_stats(1,1) + raw_stats(1,2);
    end
  end
  stats.name = folder_name;
  stats.base = base;
  stats.spine = spine;
  stats.obj = obj;
  stats.arm_abs = arm_abs;
  stats.arm_sqr = arm_sqr;
  stats.time = time;
end

function comparison = compareMethods(m,other)
  %first compute stats on the primary method
  comparison.method.name = m.name;
  fields = fieldnames(m);
  idx = m.(fields{2}) >= 0
  for j=2:length(fields)
    idx = idx & (m.(fields{j}) >= 0)
  end
  idx
  comparison.method.num_trials = length(idx);
  comparison.method.num_success = sum(idx);
  for j=2:length(fields)
    data = m.(fields{j})(idx);
    comparison.method.(fields{j}).mean = mean(data);
    comparison.method.(fields{j}).std = std(data);
    comparison.method.(fields{j}).median = median(data);
    comparison.method.(fields{j}).min = min(data);
    comparison.method.(fields{j}).max = max(data);
  end
  
  %now compute ratio stats comparing the primary to other methods
  for i=1:length(other)
    c.name = other(i).name;

    %get the valid indicies (the trials where both succeeded)
    idx = m.(fields{2}) >= 0;
    for j=2:length(fields)
      if ~isfield(other(i),fields{j})
        continue;
      end
      idx = idx & (m.(fields{j}) >= 0) & (other(i).(fields{j}) >= 0);
    end
    c.num_trials = length(idx);
    c.num_success = sum(idx);
    for j=2:length(fields)
      if ~isfield(other(i),fields{j})
        continue;
      end
      data = m.(fields{j})(idx);
      other_data = other(i).(fields{j})(idx);

      c.(fields{j}).m_mean = mean(data);
      c.(fields{j}).m_std = std(data);
      c.(fields{j}).m_median = median(data);
      c.(fields{j}).m_min = min(data);
      c.(fields{j}).m_max = max(data);

      c.(fields{j}).o_mean = mean(other_data);
      c.(fields{j}).o_std = std(other_data);
      c.(fields{j}).o_median = median(other_data);
      c.(fields{j}).o_min = min(other_data);
      c.(fields{j}).o_max = max(other_data);

      ratio = other_data./data;
      c.(fields{j}).ratio_mean = mean(ratio);
      c.(fields{j}).ratio_std = std(ratio);
      c.(fields{j}).ratio_median = median(ratio);
      c.(fields{j}).ratio_min = min(ratio);
      c.(fields{j}).ratio_max = max(ratio);
    end
    comparison.other(i) = c;
  end
end



