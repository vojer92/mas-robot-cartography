import csv


max_steps=1500 #TODO: Has to be defined

def run_batch(param_dict, model_class, output_file='results.csv'):
    keys = list(param_dict.keys())
    num_runs = len(param_dict[keys[0]])

    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(keys + ['steps_run', 'result'])

        for i in range(num_runs):
            params = {k: param_dict[k][i] for k in keys}
            model = model_class(**params)

            step = 0
            while model.running and step < max_steps:
                model.step()
                step += 1

            result = model.get_results() if hasattr(model, 'get_results') else getattr(model, 'final_result', None)
            writer.writerow([params[k] for k in keys] + [step, result])