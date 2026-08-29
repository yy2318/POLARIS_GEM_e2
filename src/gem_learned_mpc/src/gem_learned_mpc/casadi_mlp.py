def symbolic_delta(ca,q,data):
 im=ca.DM(data['input_mean']);isd=ca.DM(data['input_std']);om=ca.DM(data['output_mean']);osd=ca.DM(data['output_std']);h=(q-im)/isd
 for i,l in enumerate(data['layers']):
  h=ca.mtimes(ca.DM(l['weight']),h)+ca.DM(l['bias'])
  if i<len(data['layers'])-1:h=ca.tanh(h)
 return h*osd+om
