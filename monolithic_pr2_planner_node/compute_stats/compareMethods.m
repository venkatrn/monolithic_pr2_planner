function comparison = compareMethods(m,other)
  %first compute stats on the primary method
  comparison.method.name = m.name;
  fields = fieldnames(m);
  idx = m.(fields{2}) >= 0;
  for j=2:length(fields)
    idx = idx & (m.(fields{j}) >= 0);
  end
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


