function comparison = computeStats(path)
  num = 10;

  % prm_stats = computeMethodStats([path '/prm_'],num,0)
  % rrt_stats = computeMethodStats([path '/rrt_'],num,0)
  % rrtstar_stats = computeMethodStats([path '/rrtstar_'],num,0)
  ara_stats = computeMethodStats([path '/ara_'],num,1)
  imha_stats = computeMethodStats([path '/imha_'],num,1)
  smha_stats = computeMethodStats([path '/smha_'],num,1)
  mpwa_stats = computeMethodStats([path '/mpwa_'],num,1)
  mhg_reex_stats = computeMethodStats([path '/mhg_reex_'],num,1)
  mhg_no_reex_stats = computeMethodStats([path '/mhg_no_reex_'],num,1)
  ees_stats = computeMethodStats([path '/ees_'],num,1)

  other_methods = [mpwa_stats mhg_reex_stats mhg_no_reex_stats ees_stats imha_stats ara_stats];
  %other_methods = [cbirrt_stats multi_ompl_stats];

  comparison = compareMethods(smha_stats,other_methods);

  %displayComparison(comparison);
end