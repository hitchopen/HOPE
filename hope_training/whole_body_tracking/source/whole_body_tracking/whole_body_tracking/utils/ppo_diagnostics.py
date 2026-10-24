"""Detached PPO diagnostics; no gradients, random draws or optimizer changes."""
import torch


def value_statistics(values, targets):
    with torch.no_grad():
        error = targets-values
        variance = targets.var(unbiased=False)
        valid = variance > 1e-12
        normalized = error.square().mean()/variance.clamp_min(1e-12)
        explained = 1-error.var(unbiased=False)/variance.clamp_min(1e-12)
        return dict(critic_target_std=variance.sqrt(), critic_mse=error.square().mean(),
                    critic_mse_normalized=torch.where(valid, normalized, 0.),
                    critic_explained_variance=torch.where(valid, explained, 0.),
                    critic_variance_valid=valid.float())


def gradient_statistics(actor_parameters, critic_parameters, all_parameters, max_norm):
    with torch.no_grad():
        def norm(parameters):
            terms = [p.grad.square().sum() for p in parameters if p.grad is not None]
            return torch.stack(terms).sum().sqrt() if terms else next(iter(parameters)).new_zeros(())
        actor, critic, total = norm(actor_parameters), norm(critic_parameters), norm(all_parameters)
        return torch.stack((actor, critic, total, (max_norm/(total+1e-6)).clamp_max(1)))
