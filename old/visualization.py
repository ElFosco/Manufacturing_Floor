import pandas as pd
import plotly.express as px

def visulize_schedule(model_ilp):
    location = "./images/job_schedule.png"
    jobs = model_ilp.get_jobs()
    info = model_ilp.get_info()
    start, duration, job_resource, _ = model_ilp.get_dv()
    info_resource = model_ilp.get_info_resource()
    to_show = []
    dict_job_start = {}

    for index in range(len(start)):
        dict_job_start[index] = start[index].value()

    for job,job_info in jobs.items():
        subject = info[job_info['__id__']][0]
        start_job = start[subject].value()
        end_job = start[subject].value() + duration[subject].value()
        ris_id = job_resource[subject].value()
        resource = next((k for k, v in info_resource.items() if v == ris_id), None)
        product = info[job_info['__id__']][2]
        job_name = info[job_info['__id__']][4]
        to_show.append(dict(Task = f'Task {subject}', Start= start_job,End = end_job, Resource = resource, Product = product, Name=job_name))

    # Convert the tasks to a DataFrame for easier manipulation
    df = pd.DataFrame(to_show)

    # Calculate the duration of each task
    df['Duration'] = df['End'] - df['Start']

    # Use px.bar to create the Gantt chart with item-based colors
    fig = px.bar(df, x='Duration', y='Resource', color='Product', orientation='h',
                 hover_data=['Start', 'End', 'Task','Name'], base='Start', title="Job Shop Schedule - No State change for Cell 0 Staubli")

    # Update the layout to set the background to black and adjust text/grid colors for visibility
    fig = px.bar(
        df,
        x='Duration',
        y='Resource',
        color='Product',
        orientation='h',
        hover_data=['Start', 'End', 'Task', 'Name'],
        base='Start',
        title="Job Shop Schedule - No State change for Cell 0 Staubli",
        color_discrete_sequence=px.colors.qualitative.Set1  # vivid and clear color palette
    )

    fig.update_layout(
        plot_bgcolor='black',
        paper_bgcolor='black',
        xaxis=dict(
            showgrid=True,
            gridcolor='white',
            title="Time",
            color='white'
        ),
        yaxis=dict(
            showgrid=True,
            gridcolor='white',
            title='',
            color='white',
            categoryorder='category ascending'
        ),
        font=dict(color='white'),
        title_font=dict(color='white')
    )
    fig.write_image(location, width=1200, height=800)
    return location