function comparison = computeStats(path)
  num = 10;

  prm_stats = computeMethodStats([path '/prm_'],num,0)
  rrt_stats = computeMethodStats([path '/rrt_'],num,0)
  rrtstar_stats = computeMethodStats([path '/rrtstar_'],num,0)
  ara_stats = computeMethodStats([path '/ara_'],num,1)
  imha_stats = computeMethodStats([path '/imha_'],num,1)
  smha_stats = computeMethodStats([path '/smha_'],num,1)

  other_methods = [prm_stats rrt_stats rrtstar_stats ara_stats];
  %other_methods = [cbirrt_stats multi_ompl_stats];

  comparison = compareMethods(mha_stats,other_methods);

  %displayComparison(comparison);
end